cmake_minimum_required(VERSION 3.8)
project(jt128_sim_relay)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(jt128_sim_relay_node src/jt128_sim_relay_node.cpp)
ament_target_dependencies(jt128_sim_relay_node rclcpp sensor_msgs)

install(TARGETS jt128_sim_relay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
