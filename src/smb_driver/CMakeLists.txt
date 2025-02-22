cmake_minimum_required(VERSION 3.8)
project(smb_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED)

# Source files
set(SERIALRT_SRC
    src/RoboteqDevice.cpp
    src/SmbController.cpp
    src/auxiliaries/interProcessCommunication.cpp
)

# Create library
add_library(serialMB ${SERIALRT_SRC})

ament_target_dependencies(
  serialMB
  rclcpp
  std_msgs
  geometry_msgs
  Boost
)

# Include directories
target_include_directories(serialMB
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<INSTALL_INTERFACE:test>)


# Create executables
add_executable(testSerial test/SerialTest.cpp)
target_link_libraries(testSerial serialMB)
ament_target_dependencies(testSerial rclcpp std_msgs Boost)

add_executable(stopMB test/StopMB.cpp)
target_link_libraries(stopMB serialMB)
ament_target_dependencies(stopMB rclcpp std_msgs Boost)

add_executable(testController test/ControllerTest.cpp)
target_link_libraries(testController serialMB)
ament_target_dependencies(testController rclcpp std_msgs Boost)

# Install targets
install(TARGETS serialMB testSerial stopMB testController
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(serialMB)
ament_package()