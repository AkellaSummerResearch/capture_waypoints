cmake_minimum_required(VERSION 2.8.3)
project(capture_waypoints)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
)

find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(capture_waypoints src/capture_waypoints.cpp
                                 src/helper.cpp
                                 src/msg_conversions.cpp
                                 src/visualization_functions.cpp)
target_link_libraries(capture_waypoints ${catkin_LIBRARIES})