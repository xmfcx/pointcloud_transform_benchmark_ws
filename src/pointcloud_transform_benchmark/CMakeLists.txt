cmake_minimum_required(VERSION 3.8)
project(pointcloud_transform_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -march=native -Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(benchmark src/benchmark.cpp)

target_link_libraries(benchmark
  ${PCL_LIBRARIES}
)

ament_target_dependencies(benchmark
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  pcl_conversions
)

install(TARGETS benchmark
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

