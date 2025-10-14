cmake_minimum_required(VERSION 3.10)
project(GlobalShutterCamera)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/libcamera/libcamera)

add_executable(camera_app TestCameraFeed.cpp)
target_link_libraries(camera_app ${OpenCV_LIBS} libcamera)
