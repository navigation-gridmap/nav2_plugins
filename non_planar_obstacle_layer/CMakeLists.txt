cmake_minimum_required(VERSION 3.5)
project(non_planar_obstacle_layer)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs)
find_package(tf2_geometry_msgs)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(tf2)
find_package(grid_map_msgs)
find_package(grid_map_ros)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    sensor_msgs
    tf2_geometry_msgs
    geometry_msgs
    tf2_ros
    tf2
    grid_map_msgs
    grid_map_ros
    pcl_conversions
    pcl_ros
    tf2_eigen)

# === Build ===

add_library(${lib_name} SHARED
            src/ObservationSource.cpp
            src/non_planar_obstacle_layer.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs non_planar_obstacle_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d non_planar_obstacle_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
