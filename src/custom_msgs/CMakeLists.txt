cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)


find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Control.msg
   ControlStamped.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   custom_msgs
 )



