# QTrobot ROS interface service and message headers
This folder contains the message and service headers for QTrobot ROS interfaces. 
If you need to use QTrobot interfaces via another machine (e.g. your own laptop) you need to let your ROS knows about the message types.
Please follow the installation steps:

## Installation 
Clone [QTrobot Software](https://github.com/luxai-qtrobot/software) repository. Please notice that on the QTPC of your robot, it is already located in `~/robot/code/software`!

```
$ cd ~/
$ git clone https://github.com/luxai-qtrobot/software.git
``` 

Copy the `headers` folder or make a link in your Catkin source workspace:

```
$ cd ~/catkin_ws/src
$ ln -s ~/software/headers ./
```

Build the messages in your Catkin worksapce 
```
$ cd ~/catkin_ws
$ catkin_make --pkg qt_robot_interface qt_motors_controller qt_gesture_controller qt_vosk_app qt_nuitrack_app
```


## Testing 
now you can check the installation. 
Follow this instruction to setup your ROS environment if you have not done it yet: [Configure ROS environment for QTrobot](https://docs.luxai.com/user-manual/#configure-ros-environment-for-qtrobot)
Then connect your machine (e.g. laptop) to QTrobot WIFI and try some of the interfaces: 

```

$ rosservice call /qt_robot/speech/say "message: 'Hello.'"
```

