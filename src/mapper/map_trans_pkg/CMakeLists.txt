cmake_minimum_required(VERSION 3.8)
project(map_trans_pkg)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 禁用PCL的预编译头以减少警告
set(PCL_NO_PRECOMPILE ON CACHE BOOL "Disable PCL precompiled headers")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS2核心
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# ROS2消息
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# TF2
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# PCL相关
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)

# 其他
find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables from the package directory
install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install scripts
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



# 安装Python可执行节点
install(PROGRAMS
  map_trans_pkg/map_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_publisher_node
)

install(PROGRAMS
  map_trans_pkg/pcd_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pcd_publisher_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()