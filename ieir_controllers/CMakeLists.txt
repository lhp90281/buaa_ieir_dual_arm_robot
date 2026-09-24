cmake_minimum_required(VERSION 3.8)
project(ieir_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pinocchio REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(w3_robot_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# ============================================================================
# 1. Build the pure algorithm library (ieir_dynamics)
# ============================================================================
add_library(ieir_dynamics SHARED
  src/ieir_dynamics.cpp
)

target_include_directories(ieir_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ieir_dynamics pinocchio::pinocchio)

# Export the library for use by other packages
ament_export_targets(ieir_dynamics_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)

install(TARGETS ieir_dynamics
  EXPORT ieir_dynamics_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 2. Build ros2_control plugins
# ============================================================================

# Gravity Compensation Controller Plugin
add_library(gravity_compensation_controller SHARED
  src/gravity_compensation_controller.cpp
)

target_include_directories(gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gravity_compensation_controller 
  ieir_dynamics
  pinocchio::pinocchio
  yaml-cpp
)

ament_target_dependencies(gravity_compensation_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  sensor_msgs
)

# Joint Impedance Controller Plugin
add_library(joint_impedance_controller_plugin SHARED
  src/joint_impedance_controller_plugin.cpp
)

target_include_directories(joint_impedance_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(joint_impedance_controller_plugin 
  ieir_dynamics
  pinocchio::pinocchio
)

ament_target_dependencies(joint_impedance_controller_plugin
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  sensor_msgs
)

# Joint Position Controller Plugin (DM MIT-mode joint-space PD via motor)
add_library(joint_position_controller SHARED
  src/joint_position_controller.cpp
)

target_include_directories(joint_position_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(joint_position_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  trajectory_msgs
)

# Cartesian Position Controller Plugin
add_library(cartesian_position_controller_plugin SHARED
  src/cartesian_position_controller_plugin.cpp
)

target_include_directories(cartesian_position_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(cartesian_position_controller_plugin 
  ieir_dynamics
  pinocchio::pinocchio
)

ament_target_dependencies(cartesian_position_controller_plugin
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  sensor_msgs
  geometry_msgs
)

# DM Hardware Interface (W3 SocketCAN <-> ros2_control)
add_library(dm_hardware_interface SHARED
  src/dm_hardware_interface.cpp
)

target_include_directories(dm_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dm_hardware_interface
  yaml-cpp
)

ament_target_dependencies(dm_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  w3_robot_bridge
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ieir_controllers.xml)

# Export hardware interface plugins
pluginlib_export_plugin_description_file(hardware_interface dm_hardware_interface.xml)

install(TARGETS 
  gravity_compensation_controller
  joint_impedance_controller_plugin
  joint_position_controller
  cartesian_position_controller_plugin
  dm_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 3. Gripper Controller (standalone node)
# ============================================================================
add_executable(gripper_controller_node
  src/gripper_controller_node.cpp
)

target_link_libraries(gripper_controller_node
  yaml-cpp
)

ament_target_dependencies(gripper_controller_node
  rclcpp
  std_msgs
  w3_robot_bridge
)

install(TARGETS gripper_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts (if they exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gripper_keyboard_control.py")
  install(PROGRAMS scripts/gripper_keyboard_control.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME gripper_keyboard_control
  )
endif()

install(PROGRAMS scripts/cartesian_keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cartesian_keyboard_control
)

install(PROGRAMS scripts/friction_identification.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME friction_identification
)

install(PROGRAMS scripts/friction_compensation_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME friction_compensation_demo
)

install(PROGRAMS scripts/joint_zero_calibration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_zero_calibration
)
install(FILES scripts/calibration_direction.py scripts/calibration_seek.py
  scripts/joint_zero_calibration.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/joint_auto_calibration.py
  DESTINATION lib/${PROJECT_NAME} RENAME joint_auto_calibration)
install(PROGRAMS scripts/joint_auto_calibration.py
  DESTINATION lib/${PROJECT_NAME} RENAME joint_manual_calibration)

install(PROGRAMS scripts/joint_state_translator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_translator
)

install(PROGRAMS scripts/zero_at_current_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zero_at_current_pose
)

install(PROGRAMS scripts/teach_replay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teach_replay
)

install(PROGRAMS scripts/go_home.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go_home
)

install(PROGRAMS scripts/teleop_joint_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_joint_bridge
)

install(PROGRAMS scripts/teleop_force_observer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_force_observer
)

# ============================================================================
# 4. Install headers and other files
# ============================================================================
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES 
  ieir_controllers.xml
  dm_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(w3_migration test/test_w3_migration.py TIMEOUT 60)
  ament_add_pytest_test(calibration_seek test/test_calibration_seek.py TIMEOUT 60)
  ament_add_pytest_test(distribution test/test_distribution.py TIMEOUT 60)
endif()

ament_package()
