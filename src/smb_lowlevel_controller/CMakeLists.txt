cmake_minimum_required(VERSION 3.8)
project(smb_lowlevel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smb_driver REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(rcutils REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(control_msgs REQUIRED)
find_package(urdf REQUIRED)

# declare the C++17 standard
add_compile_options(-std=c++17)

# include directories
include_directories(
  include
)

# declare the library
add_library(${PROJECT_NAME} src/SmbHWInterface.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  ros2_control
  ros2_controllers
  control_toolbox
  control_msgs
  rcutils
)

# declare the executable node
add_executable(${PROJECT_NAME}_node src/smb_lowlevel_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  rclcpp
  std_msgs
  ros2_control
  ros2_controllers
  control_toolbox
  control_msgs
  rcutils
)

# install targets
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()