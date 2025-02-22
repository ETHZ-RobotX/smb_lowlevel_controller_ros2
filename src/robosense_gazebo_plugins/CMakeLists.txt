cmake_minimum_required(VERSION 3.8)
project(robosense_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_robosense_laser SHARED src/GazeboRosRoboSenseLaser.cpp)
target_link_libraries(gazebo_ros_robosense_laser 
  ${GAZEBO_LIBRARIES}
  RayPlugin
)
ament_target_dependencies(gazebo_ros_robosense_laser
  ros_gz
  sensor_msgs
  rclcpp
  tf2
)

add_library(gazebo_ros_robosense_gpu_laser SHARED src/GazeboRosRoboSenseLaser.cpp)
target_link_libraries(gazebo_ros_robosense_gpu_laser
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
ament_target_dependencies(gazebo_ros_robosense_gpu_laser
  ros_gz
  sensor_msgs
  rclcpp
  tf2
)

target_compile_definitions(gazebo_ros_robosense_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_robosense_laser gazebo_ros_robosense_gpu_laser
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

ament_package()
