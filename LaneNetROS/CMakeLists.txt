cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_runtime
        message_generation
        cv_bridge
)

find_package(OpenCV REQUIRED)



## Generate messages in the 'msg' folder
 add_message_files(
         FILES
         Lane_Image.msg
 )


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS
   CATKIN_DEPENDS roscpp rospy message_runtime std_msgs sensor_msgs
#  DEPENDS system_lib
)

 target_link_libraries(
         ${catkin_LIBRARIES}
         ${OpenCV_LIBS}
 )