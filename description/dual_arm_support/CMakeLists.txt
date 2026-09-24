cmake_minimum_required(VERSION 3.8)
project(dual_arm_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖 (如果有的话)
find_package(ament_cmake REQUIRED)

# --- 关键修改：添加安装规则 ---
# 将这些文件夹安装到 install/share/dual_arm_support 下
install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 package.xml (通常自动处理，但为了保险)
ament_package()
