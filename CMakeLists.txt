cmake_minimum_required(VERSION 2.8.3)
project(mapping)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf
  rosbag
#  genmsg
)

## Generate added messages and services
# sgenerate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
 CATKIN_DEPENDS
  rosbag
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(occupancy_grid src/occupancy_grid.cpp)
add_dependencies(occupancy_grid ${catkin_EXPORTED_TARGETS})
target_link_libraries(occupancy_grid ${catkin_LIBRARIES})
#add_executable(obtain_map src/obtain_map.cpp)
#target_link_libraries(obtain_map ${catkin_LIBRARIES})
