
# QTrobot ReSpeaker App

To record the microphone data we need to run QTrobot ReSpeaker app, which is using [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0)

### Installation 

:warning: **Do not install any ROS package on QTRP via `apt` commmand**


You can skip this part if the app is already instaled ``rospack find qt_respeaker_app`` on your system (QTRP).
If you get this error ``[rospack] Error: package 'qt_respeaker_app' not found`` follow next steps.

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


#### 1. Install python3 requirements (QTRP)

Go to `qt_respeaker_app` folder and install python requirements:

```
cd ~/robot/code/software/apps/qt_respeaker_app
sudo pip3 install -r requirements.txt
```

#### 2. Clone audio_common repository (QTRP)

Clone the audio_common git repository to the home folder.

```
cd ~
git clone https://github.com/ros-drivers/audio_common.git
```

#### 2. Link to catkin workspace (QTRP)

On QTRP make a symlink in ``~/catkin_ws/src`` to ``qt_respeaker_app``:

```
cd ~/catkin_ws/src
ln -s ~/robot/code/software/apps/qt_respeaker_app .
```

From audio_common we will need just audio_common_msgs. Link just `audio_common_msgs` to catkin_ws.

```
cd ~/catkin_ws/src
ln -s ~/audio_common/audio_common_msgs .
```

#### 3. Build ROS package (QTRP)

```
cd ~/catkin_ws/
catkin_make -j2
```

### Launching QTrobot Respeaker App

:exclamation:
`qt_respeaker_app` exclusively needs to access the Respeaker mic device. This means that you need to stop any other apps 
which are accessing the microphone before launching the `qt_respeaker_app`.  For example, if `qt_vosk_app` is running, you need to disable it 
in the autodtart of QTRP (i.e. commenting the coresponding line) and reboot the robot!



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

### Run automaticliy on reboot

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


### Topics

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


## Record audio data on QTPC

### Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_respeaker` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_respeaker std_msgs rospy roscpp -D "Record microphone data"
cd tutorial_qt_respeaker/src
touch tutorial_qt_respeaker_node.py
chmod +x tutorial_qt_respeaker_node.py
```

### Code

Open the `tutorial_qt_respeaker_node.py` file and the add the following code:

```python
#!/usr/bin/env python3
import wave
import rospy
from audio_common_msgs.msg import AudioData


AUDIO_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_WIDTH = 2


def channel_callback(msg, wf):
    wf.writeframes(msg.data)

# main
if __name__ == '__main__':
    # call the relevant service
    rospy.init_node('audio_record')
    
    wf = wave.open("audio.wav", 'wb')
    wf.setnchannels(AUDIO_CHANNELS)
    wf.setsampwidth(AUDIO_WIDTH)
    wf.setframerate(AUDIO_RATE)
        
    rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, channel_callback, wf)

    print("recording...")
    rospy.spin()
    print("saving...")
    wf.close()

```

This will record processed audio for ASR from channel 0 and save it into `audio.wav`, which you can later process or listen to it.
By default we setup some tunning parameters for nosie reduction and automatic gain in `config/qt_respeaker_app.yaml` which can be modified.
If you would like to change the gain of the microphone, you just need to change the  line `AGCGAIN: 100.0` in `config/qt_respeaker_app.yaml`.



## Author

Denis Kovacevic <<denis.kovacevic@luxai.com>>

