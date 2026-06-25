cmake_minimum_required(VERSION 3.8)
project(rover_slam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ORB_SLAM3_ROOT_DIR: pass via colcon --cmake-args or set as env var
if(NOT DEFINED ORB_SLAM3_ROOT_DIR)
  set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT_DIR}")
endif()
if(NOT ORB_SLAM3_ROOT_DIR)
  message(FATAL_ERROR
    "ORB_SLAM3_ROOT_DIR not set — pass -DORB_SLAM3_ROOT_DIR=<path> or export the env var")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

find_library(ORB_SLAM3_LIB ORB_SLAM3
  PATHS ${ORB_SLAM3_ROOT_DIR}/lib NO_DEFAULT_PATH)
if(NOT ORB_SLAM3_LIB)
  message(FATAL_ERROR "libORB_SLAM3.so not found in ${ORB_SLAM3_ROOT_DIR}/lib")
endif()

add_executable(orb_slam3_node src/orb_slam3_node.cpp)

target_include_directories(orb_slam3_node PRIVATE
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(orb_slam3_node
  rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge)

target_link_directories(orb_slam3_node PRIVATE
  ${ORB_SLAM3_ROOT_DIR}/lib
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
)

target_link_libraries(orb_slam3_node
  ${ORB_SLAM3_LIB}
  DBoW2
  g2o
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${EIGEN3_LIBS}
)

install(TARGETS orb_slam3_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
