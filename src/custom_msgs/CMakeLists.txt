cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)


find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)


## Generate messages in the 'msg' folder
 add_message_files(DIRECTORY msg
   FILES
   Control.msg
   ControlStamped.msg
 )
 
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)





