cmake_minimum_required(VERSION 3.8)
project(ros2_fablab)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

ament_package()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  utils
  message_runtime
)
include_directories(include)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES can_node
 CATKIN_DEPENDS roscpp utils message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_node 
  src/can_node.cpp)


add_dependencies(can_node ${can_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(can_node
  ${catkin_LIBRARIES}
)

