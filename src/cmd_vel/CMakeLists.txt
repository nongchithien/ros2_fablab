cmake_minimum_required(VERSION 3.8)
project(cmd_vel)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utils REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(can_node src/can_node.cpp)

ament_target_dependencies(can_node rclcpp std_msgs utils)


install(TARGETS can_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
