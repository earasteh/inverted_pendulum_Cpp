cmake_minimum_required(VERSION 3.5)
project(pendulum_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# for custom message "PendulumState.msg"
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/PendulumState.msg"
  DEPENDENCIES std_msgs
)

# Add executable
add_executable(pendulum_simulator src/pendulum_simulator_node.cpp)
rosidl_target_interfaces(pendulum_simulator ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs)

# Declare dependencies on the packages that node depend on
ament_target_dependencies(pendulum_simulator rclcpp geometry_msgs visualization_msgs tf2_ros)

# Install executables
install(TARGETS
  pendulum_simulator
  controller_node
  DESTINATION lib/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

