cmake_minimum_required(VERSION 3.8)
project(smb_description)

find_package(ament_cmake)


install(DIRECTORY launch rviz urdf resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()