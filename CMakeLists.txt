cmake_minimum_required(VERSION 3.16)
project(multiwii VERSION 2.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)

add_library(multiwii_msp STATIC
  third_party/msp/src/Client.cpp
  third_party/msp/src/PeriodicTimer.cpp
  third_party/msp/src/FlightController.cpp)
target_include_directories(multiwii_msp PUBLIC third_party/msp/inc ${ASIO_INCLUDE_DIR}
  PRIVATE third_party/msp/inc/msp)
target_compile_definitions(multiwii_msp PUBLIC ASIO_STANDALONE)
target_link_libraries(multiwii_msp PUBLIC Threads::Threads)

add_executable(multiwii_node src/main.cpp src/multiwii_node.cpp)
target_include_directories(multiwii_node PRIVATE include)
ament_target_dependencies(multiwii_node
  rclcpp rcl_interfaces std_msgs geometry_msgs sensor_msgs mavros_msgs tf2 tf2_ros)
target_link_libraries(multiwii_node multiwii_msp Eigen3::Eigen)

install(TARGETS multiwii_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch cfg DESTINATION share/${PROJECT_NAME})
install(FILES third_party/msp/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses RENAME msp-LICENSE)

ament_package()
