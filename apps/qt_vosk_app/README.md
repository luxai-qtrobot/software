# QTrobot Vosk speech recognizer service for ROS

## Requirements 
Please notice that for the time being, this service works only on the Ubuntu > 20 and Rasbian OS buster. 


## Installation 
Follow the instruction to install the dependencies 

### Install portaudio 

```
sudo apt install libportaudio2
```

### Install python requirements
```
sudo pip3 install sounddevice vosk
```

### Build the messages 
make a link (or copy) to `qt_vosk_app` in catkin workspace. then:

```
$ cd ~/catkin_ws/
$ catkin_make

```

### download the language models 

In the QTrobot (with raspbian buster and newer), the language models are 
in `/home/qtrobot/robot/vosk/models/`. To add a new language, just download and unzip the model.
Then rename the model folder in ISO language format name (e.g. en_GB, fr_FR)


## Lunching QTrobot Vosk App service  and try
```
$ cd ~/catkin_ws/src/qt_vosk_app
$ python3 src/qt_vosk_app.py 
```

open another terminal and try:
```
$ rosservice call /qt_robot/speech/recognize "language: 'en_US'
options:[]
timeout: 0"
```

or give your expected words as options: 

```
$ rosservice call /qt_robot/speech/recognize "language: 'en_US'
options:[yes no maybe]
timeout: 0"
```
