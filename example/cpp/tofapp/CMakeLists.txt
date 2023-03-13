cmake_minimum_required(VERSION 3.4)
include_directories(BEFORE /usr/include/c  /usr/include/cpp)

project(tofapp)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(preview_depth)
add_executable(preview_depth preview_depth.cpp tof_sampler.cpp csv_sample_handler.cpp opencv_sample_handler )
target_link_libraries( preview_depth ArducamDepthCamera  ${OpenCV_LIBS} )

