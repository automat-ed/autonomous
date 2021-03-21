cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    robot_localization
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
  CATKIN_DEPENDS rospy robot_localization
#  DEPENDS system_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/ground_truth/ekf_global.yaml
  config/ground_truth/ekf_local.yaml
  config/noise/ekf_global.yaml
  config/noise/ekf_local.yaml
  config/navsat.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
