cmake_minimum_required(VERSION 3.8)
project(eiriarm_mujoco)

# 设置mujoco文件路径
set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco" CACHE PATH
  "MuJoCo 3.3.0 SDK (include/ and lib/); bundled SDK is Linux x86_64")
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIB_DIR ${MUJOCO_DIR}/lib/libmujoco.so.3.3.0)

# Build against the SDK, install a self-contained runtime beside ROS binaries.
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_RPATH "${MUJOCO_DIR}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
if(NOT EXISTS "${MUJOCO_LIB_DIR}" OR NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "Missing MuJoCo 3.3.0 SDK at ${MUJOCO_DIR}. See README.md; do not substitute a pip or system MuJoCo version.")
endif()
install(FILES "${MUJOCO_LIB_DIR}" DESTINATION lib)
install(FILES third_party/mujoco/THIRD_PARTY_NOTICES third_party/mujoco/LICENSE
  DESTINATION share/${PROJECT_NAME}/third_party/mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(mujoco REQUIRED) # We use local version
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# ==========================================================
# Simulate UI Node
# ==========================================================
aux_source_directory(src/demo SIMULATE_SOURCES)
aux_source_directory(third_party/lodepng SIMULATE_SOURCES)

add_executable(simulate
  ${SIMULATE_SOURCES}
  src/mujoco_simulator_main.cpp
  src/simulate_bridge.cpp
)

target_link_libraries(simulate 
  ${MUJOCO_LIB_DIR}
  glfw 
  yaml-cpp
)

target_include_directories(simulate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lodepng
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tabulate/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/demo
)

target_compile_features(simulate PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(simulate
  "rclcpp"
  "std_srvs"
  "sensor_msgs"
  "geometry_msgs"
  "trajectory_msgs"
  "controller_manager_msgs"
  "urdf"
  "kdl_parser"
  "robot_state_publisher"
  "tf2"
  "tf2_ros"
  "ament_index_cpp"
)

# ==========================================================
# 安装规则
# ==========================================================

add_executable(calibration_viewer src/calibration_viewer.cpp third_party/lodepng/lodepng.cpp)
target_compile_features(calibration_viewer PUBLIC cxx_std_17)
target_include_directories(calibration_viewer PRIVATE ${MUJOCO_INCLUDE_DIR} third_party/lodepng)
target_link_libraries(calibration_viewer ${MUJOCO_LIB_DIR} glfw)
ament_target_dependencies(calibration_viewer rclcpp sensor_msgs std_msgs ament_index_cpp)
install(TARGETS calibration_viewer DESTINATION lib/${PROJECT_NAME})

# 这里的 TARGETS 名字也要对应修改
install(
  TARGETS simulate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)

if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
