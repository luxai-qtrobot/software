cmake_minimum_required(VERSION 2.8.3)
project(qt_robot_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  speech_say.srv
  speech_stop.srv
  speech_config.srv
  audio_play.srv
  audio_stop.srv
  emotion_show.srv
  emotion_stop.srv
  emotion_look.srv
  behavior_talk_audio.srv
  behavior_talk_text.srv
  setting_setVolume.srv
  setting_upload.srv
  setting_download.srv
  setting_info.srv  
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

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
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



