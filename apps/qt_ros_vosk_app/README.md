# QTrobot Vosk speech recognition

## Requirements 
Please notice that QTrobot vosk speech recognition  
- works only on the Ubuntu > 20 and Rasbian OS buster.
- Requires permission to tune Respeaker mic array for better background noise suppression (see the installation steps)


## Installation 
Follow the instruction to install the dependencies 

### Install the required packages portaudio 

```
sudo apt install libportaudio2
sudo pip3 install pyusb sounddevice vosk
```

### Setup usb permission (for pyusb)
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

### Build the messages 
make a link (or copy) to `qt_vosk_app` in catkin workspace source directory and build it. for example, 

```
$ cd ~/catkin_ws/src
$ ln -s ~/robot/code/software/apps/qt_vosk_app ./
$ cd ~/catkin_ws/
$ catkin_make
```

### Download the language models

In the QTrobot (with raspbian buster and newer), the language models are 
in `/home/qtrobot/robot/vosk/models/`. To add a new language, just download and unzip the model.
Then rename the model folder in ISO language format name (e.g. en_GB, fr_FR)


## Lunching QTrobot Vosk App service and trying it
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

or give your expected words as options. qt_vosk_app will look into the recognized script to find one of the option and return it. For example, for the following call, if you say *"Oh yes!"*, the resturn value of the service is *"yes"*.

```
$ rosservice call /qt_robot/speech/recognize "language: 'en_US'
options:[yes no maybe]
timeout: 0"
```
