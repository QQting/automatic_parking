cmake_minimum_required(VERSION 3.5)
project(auto_dock)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_line_extraction REQUIRED)
find_package(far_docking_msgs REQUIRED)


include_directories(
  include include/
)


add_executable(pattern src/pattern.cpp)
add_executable(controller src/controller.cpp)


set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs" 
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "sensor_msgs"
  "far_docking_msgs"
  "laser_line_extraction"
)

ament_target_dependencies(pattern ${DEPENDENCIES})
ament_target_dependencies(controller ${DEPENDENCIES})


install (TARGETS  pattern
  DESTINATION lib/${PROJECT_NAME})

install (TARGETS  controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
