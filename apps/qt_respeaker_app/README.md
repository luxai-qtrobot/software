# QTrobot ReSpeaker App

To record the microphone data we need to run QTrobot ReSpeaker app, which is using [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0)

## Installation 

:warning: **Do not install any ROS package on QTRP via `apt` commmand**

You can skip this part if the app is already instaled ``rospack find qt_respeaker_app`` on your system.
If you get this error ``[rospack] Error: package 'qt_respeaker_app' not found`` follow next steps.

Please notice that this should be installed in ``/home/qtrobot/robot/code/software/apps`` folder using python3.

Get the latest version:

```
cd ~/robot/code/software
git pull
```

If the folder doesn't exsist:

```
cd ~/robot/code/
git clone https://github.com/luxai-qtrobot/software.git
```


### 1. Install requirements

#### Install the required packages portaudio 

```
sudo apt install libportaudio2
sudo pip3 install pyusb sounddevice vosk
```

#### Setup usb permission (for pyusb)

Create a file called `90-mic.rules` in `/etc/udev/rules.d/`:

```
sudo nano /etc/udev/rules.d/90-mic.rules
```

and add the following content

```
ACTION=="add", SUBSYSTEMS=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="660", GROUP="plugdev"
```

Reload udev and reboot:
```
sudo udevadm control --reload
sudo udevadm trigger
sudo reboot
```


Go to `qt_respeaker_app` folder and install python requirements:

```
cd ~/robot/code/software/apps/qt_respeaker_app
sudo pip3 install -r requirements.txt
```

### 2. Clone audio_common repository 

Clone the audio_common git repository to the home folder.

```
cd ~
git clone https://github.com/ros-drivers/audio_common.git
```


### 3. Link to catkin workspace

Make a symlink in ``~/catkin_ws/src`` to ``qt_respeaker_app``:

```
cd ~/catkin_ws/src
ln -s ~/robot/code/software/apps/qt_respeaker_app .
```

From audio_common we will need just audio_common_msgs. Link just `audio_common_msgs` to catkin_ws.

```
cd ~/catkin_ws/src
ln -s ~/audio_common/audio_common_msgs .
```

### 4. Build ROS package

```
cd ~/catkin_ws/
catkin_make -j2
```

### 5. Launching QTrobot Respeaker App

To launch qt_respeaker_app you can simply run:

```
roslaunch qt_respeaker_app qt_respeaker_app.launch
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


## Record audio data

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
