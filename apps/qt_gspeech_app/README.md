# QTrobot Vosk speech recognition using ROS audio

## Requirements 
- *Python 3*
- *qt_respeaker_app* for audio input from `/qt_respeaker_app/channel0`
- *vosk* and *vosk language models* for speech recogntion 
- *pvporcupine* for optional hotword/wakeword detection 

Please notice that wakeword detection is diabled by default in the `qt_ros_vosk_app.yaml` config file. To enable it, first you need to create a wakeord using porcppuine online console (https://picovoice.ai/platform/porcupine/) and set the corresponding API key. Afterward, everything will work offline. 


## Installation 
Install the required python packages:

```
sudo pip3 install vosk pvporcupine
```


### Build the messages 
make a link (or copy) to `qt_ros_vosk_app` in catkin workspace source directory and build it. for example, 

```
$ cd ~/catkin_ws/src
$ ln -s ~/robot/code/software/apps/qt_ros_vosk_app ./
$ cd ~/catkin_ws/
$ catkin_make
```

### Download the language models

In the QTrobot (with raspbian buster and newer), the language models are 
in `/home/qtrobot/robot/vosk/models/`. To add a new language, just download and unzip the model.
Then rename the model folder in ISO language format name (e.g. en_GB, fr_FR)


## Lunching QTrobot Vosk App service and trying it
```
$ roslaunch qt_ros_vosk_app qt_ros_vosk_app.launch
```

open another terminal and try:
```
$ rosservice call /qt_robot/speech/recognize "language: 'en_US'
options:[]
timeout: 0"
```

or give your expected words as options. qt_ros_vosk_app will look into the recognized script to find one of the option and return it. For example, for the following call, if you say *"Oh yes!"*, the resturn value of the service is *"yes"*.

```
$ rosservice call /qt_robot/speech/recognize "language: 'en_US'
options:['yes' 'no' 'maybe']
timeout: 0"
```
