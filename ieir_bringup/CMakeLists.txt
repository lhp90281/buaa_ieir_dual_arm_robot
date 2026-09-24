cmake_minimum_required(VERSION 3.8)
project(ieir_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch web
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/web_console.py
  DESTINATION lib/${PROJECT_NAME} RENAME web_console)
install(FILES scripts/web_console_core.py scripts/web_console_ros.py scripts/web_console_processes.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(web_console test/test_web_console.py)
  ament_add_pytest_test(web_processes test/test_web_processes.py)
endif()

ament_package()
