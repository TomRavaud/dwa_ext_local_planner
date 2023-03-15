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
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)  #TODO:
remove_definitions(-DDISABLE_LIBUSB-1.0)  #TODO:

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}  #TODO:
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})  #TODO:

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DWAExtPlanner.cfg
)

catkin_package()

## Declare a C++ library
add_library(dwa_ext_local_planner
  src/dwa_ext_local_planner.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(dwa_ext_local_planner
   ${catkin_LIBRARIES}
)
