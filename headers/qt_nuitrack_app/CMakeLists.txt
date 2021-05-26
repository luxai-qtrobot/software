cmake_minimum_required(VERSION 2.8.3)
project(qt_nuitrack_app)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FaceInfo.msg
  Faces.msg
  GestureInfo.msg
  Gestures.msg
  HandInfo.msg
  Hands.msg
  JointInfo.msg
  SkeletonInfo.msg
  Skeletons.msg
)


## Generate actions in the 'action' folder

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    std_msgs
    message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

