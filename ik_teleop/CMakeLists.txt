cmake_minimum_required(VERSION 3.8)
project(ik_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(trajectory_msgs REQUIRED)
find_package(bio_ik REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_teleop_messages REQUIRED)
find_package(backward_ros REQUIRED)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# src/Utils.cpp src/BIO_IK_Planner.cpp

add_executable(control_node src/Utils.cpp src/BIO_IK_Planner.cpp src/control_node.cpp)
target_include_directories(control_node
  PUBLIC include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  control_node
  "moveit_common"
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "bio_ik"
)

add_executable(teleop_msg_bridge src/teleop_msg_bridge.cpp)
target_include_directories(teleop_msg_bridge
  PUBLIC include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_msg_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teleop_msg_bridge
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "custom_teleop_messages"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
)

install(TARGETS control_node
  teleop_msg_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION lib/
)
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
