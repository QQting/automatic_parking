cmake_minimum_required(VERSION 2.8.3)
project(autodock_gazebo)

find_package(catkin REQUIRED COMPONENTS
  #roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pattern
#  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include include/
)
