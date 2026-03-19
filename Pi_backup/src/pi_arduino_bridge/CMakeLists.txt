cmake_minimum_required(VERSION 3.8)
project(mren203_arduino_bridge) #this is what the project will be named

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(serial REQUIRED)

add_executable(bridge src/bridge.cpp) #tells compiler to turn the bridge.cpp node into bridge program

#include the header file 
target_include_directories(bridge PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(bridge rclcpp sensor_msgs std_msgs serial) #tells compiler to link the libraries to the bridge program

#install the node
install(TARGETS bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()