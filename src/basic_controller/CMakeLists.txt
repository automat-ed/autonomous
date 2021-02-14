cmake_minimum_required(VERSION 3.0.2)
project(basic_controller)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    custom_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS rospy custom_msgs sensor_msgs std_msgs geometry_msgs)

catkin_install_python(PROGRAMS src/control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
