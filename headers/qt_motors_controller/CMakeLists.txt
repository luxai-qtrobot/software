cmake_minimum_required(VERSION 2.8.3)
project(qt_motors_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    message_generation
    message_runtime
    roscpp
    rospy
    std_msgs)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  set_control_mode.srv
  set_velocity.srv
  home.srv
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



