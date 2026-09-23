cmake_minimum_required(VERSION 3.16)
project(w3_robot_bridge C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── ROS2 message & service definitions ──────────────────────────
set(MSG_FILES
  msg/MotorCommand.msg
  msg/MotorCommandArray.msg
  msg/MotorState.msg
  msg/MotorStateArray.msg
)
set(SRV_FILES
  srv/EnableMotor.srv
  srv/DisableMotor.srv
  srv/SetZero.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs builtin_interfaces
)

# ── Core library (zero ROS dependencies) ────────────────────────
include_directories(include ${yaml-cpp_INCLUDE_DIRS})

add_library(w3_robot_bridge_core SHARED
  src/motor_config.cpp
  src/damiao_protocol.cpp
  src/tigertoe_protocol.cpp
  src/can_channel.cpp
  src/tigertoe_channel.cpp
  src/motor_bridge.cpp
)
ament_target_dependencies(w3_robot_bridge_core
  rclcpp ament_index_cpp yaml-cpp std_msgs
)
target_link_libraries(w3_robot_bridge_core yaml-cpp)
target_compile_definitions(w3_robot_bridge_core PRIVATE _GNU_SOURCE)

# ── Node executable ──────────────────────────────────────────────
add_executable(w3_robot_bridge_node src/w3_robot_bridge_node.cpp)
target_link_libraries(w3_robot_bridge_node w3_robot_bridge_core)
ament_target_dependencies(w3_robot_bridge_node
  rclcpp ament_index_cpp yaml-cpp std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(w3_robot_bridge_node ${cpp_typesupport_target})

# ── Install ──────────────────────────────────────────────────────
install(TARGETS w3_robot_bridge_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS w3_robot_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(w3_robot_bridge_core)
ament_export_dependencies(rclcpp yaml-cpp std_msgs ament_index_cpp rosidl_default_runtime)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_damiao_profiles test/test_damiao_profiles.cpp)
  target_link_libraries(test_damiao_profiles w3_robot_bridge_core)
  add_test(NAME damiao_profiles
    COMMAND test_damiao_profiles ${CMAKE_CURRENT_SOURCE_DIR}/config)
endif()

ament_package()
