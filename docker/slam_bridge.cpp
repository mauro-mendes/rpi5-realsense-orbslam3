/**
 * slam_bridge — runs inside Docker container.
 *
 * Receives RGB+Depth frames from host via ZMQ (:5571),
 * feeds them to ORB-SLAM3 (RGBD mode, headless),
 * publishes pose JSON back to host via ZMQ (:5570).
 *
 * Architecture:
 *   realsense_producer.py  →  ZMQ PUB :5571  →  [slam_bridge]
 *   [slam_bridge]          →  ZMQ PUB :5570  →  slam_adapter.py
 *
 * Message format (incoming, 3 parts):
 *   part 0: JSON header {"width":640,"height":480,"timestamp":...,"depth_scale":0.001}
 *   part 1: raw RGB uint8  (width * height * 3 bytes)
 *   part 2: raw depth uint16 (width * height * 2 bytes)
 *
 * Message format (outgoing):
 *   JSON: {"x":...,"y":...,"z":...,"yaw":...,"timestamp":...,"state":"OK"|"LOST"}
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <GL/glew.h>  // must be before any Pangolin/ORB-SLAM3 includes

#include <zmq.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>   // ORB_SLAM3 (pulls in Viewer.h → Pangolin → gl.hpp)

static const char* VOCAB_PATH    = "/ORB_SLAM3/Vocabulary/ORBvoc.txt";
static const char* SETTINGS_PATH = "/D435i_RGBD.yaml";

// ------------------------------------------------------------------
// Minimal JSON builder (avoids nlohmann/rapidjson dependency)
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

// Minimal JSON double field parser (no full JSON lib needed)
static double parse_double(const std::string& json, const std::string& key)
{
    std::string search = "\"" + key + "\":";
    auto pos = json.find(search);
    if (pos == std::string::npos) return 0.0;
    return std::stod(json.substr(pos + search.size(), 24));
}

// ------------------------------------------------------------------
// Sophus SE3f → (x, y, z, yaw)
// ------------------------------------------------------------------
static void extract_pose(const Sophus::SE3f& Tcw,
                          float& x, float& y, float& z, float& yaw)
{
    // Camera-in-world = inverse of world-in-camera (Tcw)
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f t = Twc.translation();
    x = t[0];
    y = t[1];
    z = t[2];
    // Yaw from rotation matrix (rotation around Y axis, standard SLAM convention)
    Eigen::Matrix3f R = Twc.rotationMatrix();
    yaw = std::atan2(R(2, 0), R(0, 0));
}

// ------------------------------------------------------------------
int main(int argc, char** argv)
{
    bool use_viewer = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--viewer") == 0) use_viewer = true;
    }

    std::cout << "slam_bridge: loading ORB-SLAM3"
              << (use_viewer ? " (viewer ON)" : " (headless)") << "..." << std::endl;

    ORB_SLAM3::System SLAM(VOCAB_PATH, SETTINGS_PATH,
                            ORB_SLAM3::System::RGBD,
                            use_viewer);

    std::cout << "slam_bridge: ORB-SLAM3 ready" << std::endl;

    // ZMQ context
    void* zmq_ctx = zmq_ctx_new();

    // Subscribe to frames from host producer on :5571
    void* sub = zmq_socket(zmq_ctx, ZMQ_SUB);
    zmq_connect(sub, "tcp://127.0.0.1:5571");
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);

    // Publish pose to host slam_adapter on :5570
    void* pub = zmq_socket(zmq_ctx, ZMQ_PUB);
    zmq_bind(pub, "tcp://127.0.0.1:5570");

    std::cout << "slam_bridge: listening on :5571, publishing on :5570" << std::endl;

    int frame_count = 0;

    while (true)
    {
        // Receive 3-part frame
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
        double ts    = parse_double(hdr_str, "timestamp");
        int    width = (int)parse_double(hdr_str, "width");
        int    height= (int)parse_double(hdr_str, "height");

        if (width <= 0 || height <= 0) {
            zmq_msg_close(&msg_color);
            zmq_msg_close(&msg_depth);
            continue;
        }

        // Reconstruct images (zero-copy from ZMQ buffers)
        cv::Mat color_rgb(height, width, CV_8UC3, zmq_msg_data(&msg_color));
        cv::Mat depth_raw(height, width, CV_16UC1, zmq_msg_data(&msg_depth));

        // ORB-SLAM3 expects BGR (Camera.RGB:1 makes it convert internally,
        // but safest is to convert here and set Camera.RGB:0)
        cv::Mat color_bgr;
        cv::cvtColor(color_rgb, color_bgr, cv::COLOR_RGB2BGR);

        // Depth: keep as uint16 raw → DepthMapFactor=1000 in YAML handles conversion
        cv::Mat depth_float;
        depth_raw.convertTo(depth_float, CV_32F);

        zmq_msg_close(&msg_color);
        zmq_msg_close(&msg_depth);

        // Track
        Sophus::SE3f Tcw = SLAM.TrackRGBD(color_bgr, depth_float, ts);

        int tracking_state = SLAM.GetTrackingState();
        // 2 = OK, 3 = RECENTLY_LOST, 4 = LOST
        bool ok = (tracking_state == 2 || tracking_state == 3);

        float x = 0, y = 0, z = 0, yaw = 0;
        if (ok) extract_pose(Tcw, x, y, z, yaw);

        const char* state_str = ok ? "OK" : "LOST";
        std::string json = make_pose_json(x, y, z, yaw, ts, state_str);
        zmq_send(pub, json.c_str(), json.size(), 0);

        frame_count++;
        if (frame_count % 30 == 0) {
            std::printf("slam_bridge: %d frames | %s | pos=(%.2f, %.2f, %.2f) yaw=%.2f\n",
                        frame_count, state_str, x, y, z, yaw);
            std::fflush(stdout);
        }
    }

    // Save trajectory (TUM format: timestamp tx ty tz qx qy qz qw)
    SLAM.SaveKeyFrameTrajectoryTUM("/output/KeyFrameTrajectory.txt");
    std::cout << "slam_bridge: trajectory saved to /output/KeyFrameTrajectory.txt" << std::endl;

    SLAM.Shutdown();
    zmq_close(sub);
    zmq_close(pub);
    zmq_ctx_destroy(zmq_ctx);
    return 0;
}
