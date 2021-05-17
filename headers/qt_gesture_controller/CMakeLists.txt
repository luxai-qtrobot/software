cmake_minimum_required(VERSION 2.8.3)
project(qt_gesture_controller)


# Load catkin and all dependencies required for this package
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
  gesture_record.srv
  gesture_save.srv
  gesture_stop.srv
  gesture_play.srv
  gesture_list.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


# Declare catkin project
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    std_msgs
    message_runtime
)



