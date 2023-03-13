cmake_minimum_required(VERSION 3.0.2)
project(dwa_ext_local_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dwa_ext_local_planner
  src/dwa_ext_local_planner.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(dwa_ext_local_planner
   ${catkin_LIBRARIES}
)
