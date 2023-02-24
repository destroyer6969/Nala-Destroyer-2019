cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ObjectAttr.msg
   BoundingBox.msg
   BoundingBoxes.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES object_detection
  CATKIN_DEPENDS cv_bridge image_transport rospy sensor_msgs std_msgs message_runtime
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/main.py
    DESTINATION $(CATKIN_PACKAGE_SHARE_DESTINATION)
)
