
# QTrobot ReSpeaker App

This is QTrobot ReSpeaker app using [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0)

## Requirements 
Please notice that this should be installed on QTRP running with python3. 
This should be in ``/home/qtrobot/robot/code/software/apps`` folder on QTRP.

Get the latest version (ssh to QTRP):

```
cd ~/robot/code/software
git pull
```

If the folder doesn't exsist:

```
cd ~/robot/code/
git clone https://github.com/luxai-qtrobot/software.git
```

## Installation 

You can skip this part if the app is already instaled ``rospack find qt_respeaker_app`` on your system (QTRP).
If you get this error ``[rospack] Error: package 'qt_respeaker_app' not found`` follow next steps.

### 1. Install python3 requirements (QTRP)

Go to qt_respeaker_app folder and install python requirements:

```
cd ~/robot/code/software/apps/qt_respeaker_app
sudo pip3 install -r requirements.txt
```

### 2. Link to catkin workspace (QTRP)

On QTRP make a symlink in ``~/catkin_ws/src`` to ``qt_respeaker_app``:

```
cd ~/catkin_ws/src
ln -s ~/robot/code/software/apps/qt_respeaker_app .
```

### 3. Build ROS package (QTRP)

```
cd ~/catkin_ws/
catkin_make -j2
```

## Launching QTrobot Respeaker App
Go to qt_respeaker_app folder and execute ``start_qt_respeaker_app.sh``
```
cd ~/catkin_ws/src/qt_respeaker_app/autostart
./start_qt_respeaker_app.sh
```

To check that the qt_respeaker_app is running open another terminal and try:
```
rostopic list
```

You should see all qt_respeaker_app topics:

```
/qt_respeaker_app/channel0
/qt_respeaker_app/channel1
/qt_respeaker_app/channel2
/qt_respeaker_app/channel3
/qt_respeaker_app/channel4
/qt_respeaker_app/channel5
/qt_respeaker_app/is_speaking
/qt_respeaker_app/sound_direction
```

## Topics

QTrobot ReSpeaker app extracts data from [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0) and publishes to following topics:

Publisher:

``/qt_respeaker_app/channel0`` 
Channel 0: processed audio for ASR

``/qt_respeaker_app/channel1``
Channel 1: mic1 raw data

``/qt_respeaker_app/channel2``
Channel 2: mic2 raw data

``/qt_respeaker_app/channel3``
Channel 3: mic3 raw data

``/qt_respeaker_app/channel4``
Channel 4: mic4 raw data

``/qt_respeaker_app/channel5``
Channel 5: merged playback

``/qt_respeaker_app/is_speaking``
VAD (Voice Activity Detection)

``/qt_respeaker_app/sound_direction``
DOA (Direction of Arrival)


## Run automaticliy on reboot

Copy autostart script to QTRP autostart folder:

```
cp ~/robot/code/software/apps/qt_respeaker_app/autostart/start_qt_respeaker_app.sh ~/robot/autostart
```

Edit ``autostart_screens.sh``:

```
sudo nano ~/robot/autostart/autostart_screens.sh
```

Add this line below other scripts 

```
run_script "start_qt_respeaker_app.sh"
```


## Author

Denis Kovacevic <<denis.kovacevic@luxai.com>>

