# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc
SCRIPT_NAME="start_qt_vosk_app"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 180

read -d '' SPEECHENV << EOF
export PYTHONPATH="${PYTHONPATH}:/home/qtrobot/catkin_ws/devel/lib/python3/dist-packages";
python3 /home/qtrobot/catkin_ws/src/qt_vosk_app/src/qt_vosk_app_node.py;
EOF

exec echo "$QT_PASS" | sudo -kSi bash -c "$SPEECHENV"

} #&>> ${LOG_FILE}
