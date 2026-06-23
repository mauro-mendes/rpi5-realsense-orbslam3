/**
 * slam_bridge — runs inside Docker container (RGBD-Inertial mode).
 *
 * Receives RGB+Depth frames via ZMQ :5571 and IMU samples via ZMQ :5572
 * from realsense_producer.py on the host.
 * Feeds them to ORB-SLAM3 (IMU_RGBD mode).
 * Publishes pose JSON back to host via ZMQ :5570.
 *
 * Architecture:
 *   realsense_producer.py  →  ZMQ :5571 (video)  →  [slam_bridge]
 *   realsense_producer.py  →  ZMQ :5572 (IMU)    →  [slam_bridge]
 *   [slam_bridge]          →  ZMQ :5570 (pose)   →  slam_adapter.py
 *
 * Video message format (3 parts):
 *   part 0: JSON header {"width":640,"height":480,"timestamp":...,"depth_scale":0.001}
 *   part 1: raw RGB uint8  (width * height * 3 bytes)
 *   part 2: raw depth uint16 (width * height * 2 bytes)
 *
 * IMU message format (single JSON):
 *   {"type":"gyro"|"accel", "x":..., "y":..., "z":..., "t":...}
 *   Timestamps must be from the same RealSense hardware clock as video frames.
 *
 * IMU handling:
 *   Gyro arrives at 400Hz, accel at 200Hz. The IMU thread buffers both.
 *   For each video frame, accel is linearly interpolated to match gyro timestamps.
 *   The resulting vector of IMU::Point (at ~400Hz) is passed to TrackRGBD().
 *
 * Pose message format (outgoing):
 *   JSON: {"x":...,"y":...,"z":...,"yaw":...,"timestamp":...,"state":"OK"|"LOST"}
 */

#include <atomic>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

#include <epoxy/gl.h>  // must be before any Pangolin/ORB-SLAM3 includes

#include <zmq.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>     // ORB_SLAM3 (includes ImuTypes.h transitively)
#include <ImuTypes.h>   // ORB_SLAM3::IMU::Point

static const char* VOCAB_PATH    = "/ORB_SLAM3/Vocabulary/ORBvoc.txt";
static const char* SETTINGS_PATH = "/D435i_RGBD.yaml";

// ------------------------------------------------------------------
// Minimal JSON helpers (avoids nlohmann/rapidjson dependency)
// ------------------------------------------------------------------
static std::string make_pose_json(float x, float y, float z, float yaw,
                                   double ts, const char* state)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"yaw\":%.4f,"
        "\"timestamp\":%.6f,\"state\":\"%s\"}",
        x, y, z, yaw, ts, state);
    return std::string(buf);
}

static double parse_double(const std::string& json, const std::string& key)
{
    std::string search = "\"" + key + "\":";
    auto pos = json.find(search);
    if (pos == std::string::npos) return 0.0;
    return std::stod(json.substr(pos + search.size(), 32));
}

// ------------------------------------------------------------------
// Sophus SE3f → (x, y, z, yaw)
// ------------------------------------------------------------------
static void extract_pose(const Sophus::SE3f& Tcw,
                          float& x, float& y, float& z, float& yaw)
{
    Sophus::SE3f Twc = Tcw.inverse();  // camera-in-world
    Eigen::Vector3f t = Twc.translation();
    x = t[0]; y = t[1]; z = t[2];
    Eigen::Matrix3f R = Twc.rotationMatrix();
    yaw = std::atan2(R(2, 0), R(0, 0));
}

// ------------------------------------------------------------------
// IMU buffering (gyro 400Hz + accel 200Hz → interpolated IMU::Point)
// ------------------------------------------------------------------
struct ImuSample { float x, y, z; double t; };

static std::mutex              g_imu_mutex;
static std::deque<ImuSample>   g_gyro_buf;
static std::deque<ImuSample>   g_accel_buf;

// IMU receiver thread: drains :5572 and fills the buffers.
static void imu_recv_loop(void* sock)
{
    while (g_running) {
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        // Blocking with RCVTIMEO=100ms so the thread can exit cleanly
        int rc = zmq_msg_recv(&msg, sock, 0);
        if (rc < 0) { zmq_msg_close(&msg); continue; }

        std::string s((char*)zmq_msg_data(&msg), zmq_msg_size(&msg));
        zmq_msg_close(&msg);

        bool is_gyro = (s.find("\"gyro\"") != std::string::npos);
        ImuSample sample;
        sample.x = (float)parse_double(s, "x");
        sample.y = (float)parse_double(s, "y");
        sample.z = (float)parse_double(s, "z");
        sample.t = parse_double(s, "t");

        std::lock_guard<std::mutex> lk(g_imu_mutex);
        if (is_gyro)
            g_gyro_buf.push_back(sample);
        else
            g_accel_buf.push_back(sample);
    }
}

// Collect IMU::Point between t_prev and t_curr (exclusive on t_prev, inclusive on t_curr).
// Gyro is the primary rate; accel is linearly interpolated at each gyro timestamp.
// Consumed samples are removed from the buffers.
static std::vector<ORB_SLAM3::IMU::Point>
collect_imu_between(double t_prev, double t_curr)
{
    std::lock_guard<std::mutex> lk(g_imu_mutex);
    std::vector<ORB_SLAM3::IMU::Point> pts;

    if (g_gyro_buf.empty() || g_accel_buf.size() < 2)
        return pts;

    for (const auto& g : g_gyro_buf) {
        if (g.t <= t_prev) continue;
        if (g.t >  t_curr) break;

        // Linearly interpolate accel at g.t using surrounding accel samples
        float ax = 0.f, ay = 0.f, az = 0.f;
        for (size_t i = 0; i + 1 < g_accel_buf.size(); i++) {
            if (g_accel_buf[i].t <= g.t && g_accel_buf[i+1].t > g.t) {
                double r = (g.t - g_accel_buf[i].t) /
                           (g_accel_buf[i+1].t - g_accel_buf[i].t);
                ax = (float)(g_accel_buf[i].x + r * (g_accel_buf[i+1].x - g_accel_buf[i].x));
                ay = (float)(g_accel_buf[i].y + r * (g_accel_buf[i+1].y - g_accel_buf[i].y));
                az = (float)(g_accel_buf[i].z + r * (g_accel_buf[i+1].z - g_accel_buf[i].z));
                break;
            }
        }
        pts.emplace_back(ax, ay, az, g.x, g.y, g.z, g.t);
    }

    // Trim gyro buffer: remove samples up to t_curr
    while (!g_gyro_buf.empty() && g_gyro_buf.front().t <= t_curr)
        g_gyro_buf.pop_front();

    // Trim accel buffer: keep one sample before t_curr for future interpolation
    while (g_accel_buf.size() >= 2 && g_accel_buf[1].t <= t_curr)
        g_accel_buf.pop_front();

    return pts;
}

// ------------------------------------------------------------------
int main(int argc, char** argv)
{
    bool use_viewer = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--viewer") == 0) use_viewer = true;
    }

    std::cout << "slam_bridge: loading ORB-SLAM3 IMU_RGBD"
              << (use_viewer ? " (viewer ON)" : " (headless)") << "..." << std::endl;

    ORB_SLAM3::System SLAM(VOCAB_PATH, SETTINGS_PATH,
                            ORB_SLAM3::System::IMU_RGBD,
                            use_viewer);

    std::cout << "slam_bridge: ORB-SLAM3 ready" << std::endl;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ZMQ context
    void* zmq_ctx = zmq_ctx_new();

    // Subscribe to video frames from host producer on :5571
    void* sub = zmq_socket(zmq_ctx, ZMQ_SUB);
    zmq_connect(sub, "tcp://127.0.0.1:5571");
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);

    // Subscribe to IMU data from host producer on :5572
    void* imu_sub = zmq_socket(zmq_ctx, ZMQ_SUB);
    zmq_connect(imu_sub, "tcp://127.0.0.1:5572");
    zmq_setsockopt(imu_sub, ZMQ_SUBSCRIBE, "", 0);
    int imu_rcvtimeo = 100;  // ms — allows g_running check on timeout
    zmq_setsockopt(imu_sub, ZMQ_RCVTIMEO, &imu_rcvtimeo, sizeof(imu_rcvtimeo));

    // Publish pose to host slam_adapter on :5570
    void* pub = zmq_socket(zmq_ctx, ZMQ_PUB);
    zmq_bind(pub, "tcp://127.0.0.1:5570");

    std::cout << "slam_bridge: video :5571 | IMU :5572 | pose :5570" << std::endl;

    // Start IMU receiver thread
    std::thread imu_thread(imu_recv_loop, imu_sub);

    int    frame_count = 0;
    double t_prev      = 0.0;  // timestamp of the previous video frame

    while (g_running)
    {
        // Receive 3-part video frame
        zmq_msg_t msg_hdr, msg_color, msg_depth;

        zmq_msg_init(&msg_hdr);
        if (zmq_msg_recv(&msg_hdr, sub, 0) < 0) { zmq_msg_close(&msg_hdr); continue; }
        std::string hdr_str((char*)zmq_msg_data(&msg_hdr), zmq_msg_size(&msg_hdr));
        zmq_msg_close(&msg_hdr);

        zmq_msg_init(&msg_color);
        if (zmq_msg_recv(&msg_color, sub, 0) < 0) { zmq_msg_close(&msg_color); continue; }

        zmq_msg_init(&msg_depth);
        if (zmq_msg_recv(&msg_depth, sub, 0) < 0) { zmq_msg_close(&msg_depth); continue; }

        // Parse header
        double ts     = parse_double(hdr_str, "timestamp");
        int    width  = (int)parse_double(hdr_str, "width");
        int    height = (int)parse_double(hdr_str, "height");

        if (width <= 0 || height <= 0) {
            zmq_msg_close(&msg_color);
            zmq_msg_close(&msg_depth);
            continue;
        }

        // Reconstruct images
        cv::Mat color_rgb(height, width, CV_8UC3, zmq_msg_data(&msg_color));
        cv::Mat depth_raw(height, width, CV_16UC1, zmq_msg_data(&msg_depth));

        cv::Mat color_bgr;
        cv::cvtColor(color_rgb, color_bgr, cv::COLOR_RGB2BGR);

        cv::Mat depth_float;
        depth_raw.convertTo(depth_float, CV_32F);

        zmq_msg_close(&msg_color);
        zmq_msg_close(&msg_depth);

        // Collect IMU measurements since previous frame (gyro interpolated with accel)
        std::vector<ORB_SLAM3::IMU::Point> imu_pts;
        if (t_prev > 0.0)
            imu_pts = collect_imu_between(t_prev, ts);
        t_prev = ts;

        // Track
        Sophus::SE3f Tcw = SLAM.TrackRGBD(color_bgr, depth_float, ts, imu_pts);

        int  tracking_state = SLAM.GetTrackingState();
        bool ok = (tracking_state == 2 || tracking_state == 3);

        float x = 0, y = 0, z = 0, yaw = 0;
        if (ok) extract_pose(Tcw, x, y, z, yaw);

        const char* state_str = ok ? "OK" : "LOST";
        std::string json = make_pose_json(x, y, z, yaw, ts, state_str);
        zmq_send(pub, json.c_str(), json.size(), 0);

        frame_count++;
        if (frame_count % 30 == 0) {
            std::printf("slam_bridge: %d frames | %s | pos=(%.2f, %.2f, %.2f) yaw=%.2f | imu=%zu pts\n",
                        frame_count, state_str, x, y, z, yaw, imu_pts.size());
            std::fflush(stdout);
        }
    }

    // Save trajectory (TUM format: timestamp tx ty tz qx qy qz qw)
    SLAM.SaveKeyFrameTrajectoryTUM("/output/KeyFrameTrajectory.txt");
    std::cout << "slam_bridge: trajectory saved to /output/KeyFrameTrajectory.txt" << std::endl;

    imu_thread.join();
    SLAM.Shutdown();
    zmq_close(sub);
    zmq_close(imu_sub);
    zmq_close(pub);
    zmq_ctx_destroy(zmq_ctx);
    return 0;
}
