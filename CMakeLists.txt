cmake_minimum_required(VERSION 3.0.2)
project(dwa_ext_local_planner)

# Add the path to the pytorch library
list(APPEND CMAKE_PREFIX_PATH "/home/tom/Traversability-Tom/libtorch")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  base_local_planner
  cv_bridge
  sensor_msgs
)

find_package(OpenCV 4.2.0 REQUIRED)

find_package(Torch REQUIRED)

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
    config/DWAExtPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwa_ext_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
)

## Declare a C++ library
add_library(dwa_ext_local_planner
  src/dwa_ext_local_planner_ros.cpp src/dwa_ext_local_planner.cpp src/traversability_cost_function.cpp
)

# add_dependencies(dwa_ext_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dwa_ext_local_planner
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TORCH_LIBRARIES}
)

# Set the optimization level to O2 and set a specific version of C++ (C++17)
target_compile_options(dwa_ext_local_planner PRIVATE -O2 -std=c++17)


# install(TARGETS dwa_ext_local_planner
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#        )

# install(FILES lp_plugin.xml
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

