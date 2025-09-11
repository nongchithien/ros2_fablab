cmake_minimum_required(VERSION 3.8)
project(utils)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/Cmdvel.msg"
  "msg/Poserobot.msg"
  "msg/Waypoint.msg"
  DEPENDENCIES std_msgs  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
