cmake_minimum_required(VERSION 3.0.2)
project(basic_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES basic_controller
   CATKIN_DEPENDS roscpp rospy std_msgs custom_msgs
 # DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


