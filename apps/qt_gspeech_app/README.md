# QTrobot Google speech recognition using ROS audio

## Requirements 
- *Python 3*
- *qt_respeaker_app* for audio input from `/qt_respeaker_app/channel0`
- google speech API credentials

### Setup instructions

#### Google account and API credentials
Please follow the setup instructions to [**setup your google account**](https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries#before-you-begin) before enabling the `qt_gspeech_app`.

Export your google api credentials in .json file and save it on the QTPC.

Edit and add to the last line in `~/.bash_aliases` file:

    export GOOGLE_APPLICATION_CREDENTIALS=`<path-to-your-file>`
    *example*:"/home/qtrobot/credentials.json"

#### Python virtualenvironment
We recommend you use [python virtual environments](https://docs.python.org/3/library/venv.html) to install all python modules and setup the google speech recognition.
1. Install python virtual environment:
    ```bash
    sudo apt install python3.8-venv
    ```
2. Navigate to 'qt_gspeech_app' folder:
    ```bash
    cd ~/robot/code/software/apps/qt_gspeech_app
    ```
3. Setup virtual environment:
    ```bash
    python3 -m venv .venv
    ```
4. To use is you just need to activate it:
    ```bash
    source .venv/bin/activate
    ```
5. install all python modules and plugins inside this virtual environment.

#### Link to catkin workspace and build

To be able to use rosservice you need to link the code to the catkin workspace and build it:
1. navigate to catkin workspace:
  ```bash
  cd ~/catkin_ws/src
  ```
2. link the qt_gspeech_app:
  ```bash
  ln -s /home/qtrobot/robot/code/software/apps/qt_gspeech_app .
  ```
3. rebuild catkin workspace:
  ```bash
  cd ~/catkin_ws && caktin_make -j4
  ```

#### Autostart
To enable it in autostart script (`~/robot/autostart/autostart_screens.sh`) on QTPC follow next steps: 
1. edit autostart_screen.sh
  ```bash
  nano ~/robot/autostart/autostart_screens.sh
  ```
  and add this line below other scripts:
  ```bash
  run_script "start_qt_gspeech_app.sh"
  ```
2. create "start_qt_gspeech_app.sh"
   ```bash
   nano ~/robot/autostart/start_qt_gspeech_app.sh
   ```
   add and edit the following content:
   ```bash
   # !/bin/bash

    source /home/qtrobot/robot/autostart/qt_robot.inc
    SCRIPT_NAME="start_qt_gspeech_app"
    LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

    {
    prepare_ros_environment
    wait_for_ros_node "/rosout" 60

    source /home/qtrobot/catkin_ws/src/qt_gspeech_app/.venv/bin/activate;
    python /home/qtrobot/catkin_ws/src/qt_gspeech_app/src/qt_gspeech_app_node.py;

    } &>> ${LOG_FILE}
   ```
  3. save the file and reboot the QTrobot


### Accessing voice recognition from terminal 
Similar to the offline version of speech recognition, the interface can be accessed using ROS Service `/qt_robot/speech/recognize` command line tools as shown bellow: 

```bash
rosservice call /qt_robot/speech/recognize "language: 'en_US'
options:
- ''
timeout: 10"
```

