cmake_minimum_required(VERSION 3.10)
project(slam_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

set(ORB_SLAM3_DIR /ORB_SLAM3)

include_directories(
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${ORB_SLAM3_DIR}/lib)

add_executable(slam_bridge slam_bridge.cpp)

target_link_libraries(slam_bridge
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${OpenCV_LIBS}
    zmq
    pthread
    -Wl,-rpath,${ORB_SLAM3_DIR}/lib
)
