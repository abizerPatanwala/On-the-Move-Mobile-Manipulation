cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ros2_grasping REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(/usr/include/python3.10)
include_directories(~/optimLib/optim/header_only_version)
include_directories(/usr/include/eigen3)
include_directories(~/gnuplot-cpp)

add_executable(globalPlanner src/HybridAStar.cpp)
ament_target_dependencies(globalPlanner rclcpp nav_msgs std_msgs)
add_executable(localPlanner src/RegulatedPurePursuit.cpp)
ament_target_dependencies(localPlanner rclcpp std_msgs nav_msgs tf2 rclcpp_action geometry_msgs)
add_executable(optimizer src/Optimizer.cpp)
ament_target_dependencies(optimizer rclcpp kdl_parser nav_msgs)
add_executable(armController src/ArmController.cpp)
ament_target_dependencies(armController rclcpp std_msgs sensor_msgs controller_manager rclcpp_action)
add_executable(gripperController src/GripperController.cpp)
ament_target_dependencies(gripperController rclcpp std_msgs sensor_msgs)
add_executable(main src/Main.cpp)
ament_target_dependencies(main rclcpp std_msgs nav_msgs geometry_msgs rclcpp_action ros2_grasping kdl_parser)
add_executable(positionServoing src/PositionServoing.cpp)
ament_target_dependencies(positionServoing rclcpp std_msgs sensor_msgs nav_msgs rclcpp_action tf2 kdl_parser) 

set(srv_files "srv/MapPath.srv" "srv/PickPoint.srv")
set(action_files "action/ArmControl.action" "action/BaseControl.action" "action/Servoing.action")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${action_files}
  DEPENDENCIES nav_msgs std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(globalPlanner "${cpp_typesupport_target}")
target_link_libraries(main "${cpp_typesupport_target}")
target_link_libraries(optimizer "${cpp_typesupport_target}")
target_link_libraries(localPlanner "${cpp_typesupport_target}")
target_link_libraries(armController "${cpp_typesupport_target}")
target_link_libraries(positionServoing "${cpp_typesupport_target}")


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

install(
  DIRECTORY launch map 
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  globalPlanner localPlanner optimizer armController gripperController main positionServoing
  DESTINATION lib/${PROJECT_NAME})
ament_package()
