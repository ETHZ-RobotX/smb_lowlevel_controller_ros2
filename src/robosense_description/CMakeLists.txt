cmake_minimum_required(VERSION 3.8)
project(robosense_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf world
        DESTINATION share/${PROJECT_NAME}
        USE_SOURCE_PERMISSIONS
)

ament_package()