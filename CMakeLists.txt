cmake_minimum_required(VERSION 2.8.3)
project(geometry_check)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  lib/uncc_rgbd_odom/core/include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
)

set(RGBD_VO_DIR lib/uncc_rgbd_odom/core)

# Build the RGBD_VO
add_library(rgbd_vo
    ${RGBD_VO_DIR}/src/rgbd_odometry_core.cpp
    ${RGBD_VO_DIR}/src/RobustMatcher.cpp
    ${RGBD_VO_DIR}/src/image_function_dev.cpp
    ${RGBD_VO_DIR}/src/opencv_function_dev.cpp
)
target_link_libraries(rgbd_vo     
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)


add_executable(test_program 
  src/test.cpp)
target_link_libraries(test_program
  ${OpenCV_LIBRARIES}
  rgbd_vo
)
