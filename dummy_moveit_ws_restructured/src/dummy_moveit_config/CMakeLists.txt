cmake_minimum_required(VERSION 3.22)
project(dummy_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(dummy_servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(dummy_servo_keyboard_input control_msgs geometry_msgs rclcpp)

add_executable(dummy_servo_cpp_interface_demo src/servo_cpp_interface_demo.cpp)
ament_target_dependencies(dummy_servo_cpp_interface_demo moveit_msgs moveit_servo rclcpp shape_msgs tf2_ros)

install(TARGETS
  dummy_servo_keyboard_input
  dummy_servo_cpp_interface_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
