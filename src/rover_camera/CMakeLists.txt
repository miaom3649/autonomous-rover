cmake_minimum_required(VERSION 3.8)
project(rover_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs)
target_include_directories(camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${LIBCAMERA_INCLUDE_DIRS})
target_link_directories(camera_node PRIVATE
  ${LIBCAMERA_LIBRARY_DIRS})
target_link_libraries(camera_node
  ${OpenCV_LIBS}
  ${LIBCAMERA_LIBRARIES})

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
