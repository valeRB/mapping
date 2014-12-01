cmake_minimum_required(VERSION 2.8.3)
project(mapping)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf
  robot_msgs
#  genmsg
)

## Generate added messages and services
# sgenerate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp tf robot_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(occupancy_grid src/occupancy_grid.cpp)
target_link_libraries(occupancy_grid ${catkin_LIBRARIES})
