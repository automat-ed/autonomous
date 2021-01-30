cmake_minimum_required(VERSION 3.0.2)
project(basic_controller)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

