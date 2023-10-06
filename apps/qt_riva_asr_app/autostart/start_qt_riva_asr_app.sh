# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="qt_riva_asr_app"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60

sleep 3
roslaunch qt_riva_asr_app qt_riva_asr_app.launch

} &>> ${LOG_FILE}
