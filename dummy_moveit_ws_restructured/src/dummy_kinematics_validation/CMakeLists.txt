cmake_minimum_required(VERSION 3.22)
project(dummy_kinematics_validation)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(ik_fk_validator src/ik_fk_validator.cpp)
target_compile_features(ik_fk_validator PUBLIC cxx_std_17)
target_include_directories(ik_fk_validator PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  ik_fk_validator
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS ik_fk_validator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
