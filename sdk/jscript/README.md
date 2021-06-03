
# qtrobot.js 
The **qtrobot.js** library is JavaScript SDK for QTrobot. It is a wrapper over [roslibjs](http://wiki.ros.org/roslibjs) which facilitate stablishing connection with QTrobot rosbridge and expose simple APIs to interact with the robot's ROS interfaces such as playing gestures, configuring the text-to-speech interface, showing emotion, etc. It also offers some generic functions to call any ROS service ([call_service][3]), publish ([publish][5]) and subscribe ([subscribe][7]) to a generic topic.

Please read [QTrobot Interface documentaion](https://docs.luxai.com/api/) for the details description of available ROS interfacesfor QTrobot.

## Quick start
First source the required JavaScript files in your HTML. Then from your JavaScript code, create an instance of `QTrobot` object with the proper URL of your running rosbridge websocket server. 

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <!-- for ROS websockets -->
    <script src="roslib.min.js"></script>
    <script src="eventemitter2.min.js"></script>
    <script src="qtrobot-1.0.min.js"></script>
</head>
<body>

<script>    
var qtrobot = null;
document.addEventListener('DOMContentLoaded', function () {
    console.log("connecting to QTrobot (please wait...)");
    qtrobot = new QTrobot({
        url : 'ws://192.168.100.2:9091',
        connection: function(){            
            console.log("connected");
            qtrobot.talk_text('Hello!', function(){
                qtrobot.show_emotion('QT/happy');
            });        
        },
        error: function(error){
            console.log(error);
        },
        close: function(){
            console.log("disconnected.");
        }
    }); //end of qtrobot

}); // end of DOMContentLoaded
</script>

</body>
</html>
```


### QTrobot class methods

*   [constructor][1]
    *   [Parameters][2]
*   [call_service][3]
    *   [Parameters][4]
*   [publish][5]
    *   [Parameters][6]
*   [subscribe][7]
    *   [Parameters][8]
*   [show_emotion][9]
    *   [Parameters][10]
*   [play_gesture][11]
    *   [Parameters][12]
*   [set_volume][13]
    *   [Parameters][14]
*   [home_motors][15]
    *   [Parameters][16]
*   [say_text][17]
    *   [Parameters][18]
*   [configure_tts][19]
    *   [Parameters][20]
*   [play_audio][21]
    *   [Parameters][22]
*   [talk_text][23]
    *   [Parameters][24]
*   [talk_audio][25]
    *   [Parameters][26]


## constructor

QTrobot constructor method
this function will automatically call the connect method with the given 'url' to the constructor

### Parameters

*   `obj` **[object][27]** an object with the following fields
    url: the rosbridge websocket url (e.g. 'ws://127.0.0.1:9090')
    reconnect_time: an optionnal delay (in ms) before traying to reconnect
    callback.connection: an optional callback to be called upon connection
    callback.close: an optional callback to be called on closing/disconnection
    callback.error: an optional callback to be called upon any error on connection

## call_service

Call a ros service

### Parameters

*   `_name` **[string][28]** service name (e.g. /qt_robot/speech/say)
*   `_type` **[string][28]** service type (e.g.  qt_robot_interface/speech_say)
*   `_params` **[object][27]** an object specifying the service’s params
*   `callback` **[function][29]** a callback to get the result (optional, default `null`)

## publish

Publish to a ros topic

### Parameters

*   `_name` **[string][28]** topic name (e.g. /qt_robot/speech/say)
*   `_type` **[string][28]** topic message type (e.g. std_msgs/String)
*   `_params` **[object][27]** object specifying the message fileds (e.g. {data: 'hello'})

## subscribe

Subscribe to a ros topic

### Parameters

*   `_name` **[string][28]** topic name (e.g. /qt_robot/joints/state)
*   `_type` **[string][28]** topic message type (e.g.  sensor_msgs/JointState)
*   `callback` **[function][29]** a callback to receive the published message (optional, default `null`)

## show_emotion

Show a facial emotion on QTrobot display

### Parameters

*   `emotion` **[string][28]** name of the emotion (e.g. QT/happy)
*   `callback` **[function][29]** a callback to be called after playing the emotion file (optional, default `null`)

## play_gesture

Play a QTrobot recorded gesture

### Parameters

*   `gesture` **[string][28]** name of a gesture (e.g. QT/happy)
*   `callback` **[function][29]** a callback to be called after playing the gesture (optional, default `null`)

## set_volume

Set QTrobot speaker volume

### Parameters

*   `volume` **[number][30]** level in percentage (0-100)
*   `callback` **[function][29]** a callback to check the result (optional, default `null`)

## home_motors

Move all QTrobot motors to home position

### Parameters

*   `callback` **[function][29]** a callback to be called after homing all motors (optional, default `null`)

## say_text

Speak a text using QTrobot TTS

### Parameters

*   `message` **[string][28]** text message
*   `callback` **[function][29]** a callback to be called after speaking out the message (optional, default `null`)

## configure_tts

Configure QTrobot TTS engine

### Parameters

*   `language` **[string][28]** speech langauge (e.g. en_US) (optional, default `'default'`)
*   `pitch` **[number][30]** speech pitch value (it may not supported for all voices) (optional, default `0`)
*   `speed` **[number][30]** speech talking speed (100 is the normal speed.) (optional, default `0`)
*   `callback` **[function][29]** a callback to check the result (optional, default `null`)

## play_audio

Play a mp3 or wav audio file

### Parameters

*   `name` **[string][28]** the name of the sudio file without extention (e.g. QT/5LittleBunnies)
*   `path` **[string][28]** an optinal path (leave it empty '' for QTrobot defualt audio path) (optional, default `''`)
*   `callback` **[function][29]** a callback to be called after playing the audio file (optional, default `null`)

## talk_text

Speak a text using QTrobot TTS with facial emotion (moving mouth)

### Parameters

*   `message` **[string][28]** text message
*   `callback` **[function][29]** a callback to be called after speaking out the message (optional, default `null`)

## talk_audio

Play a mp3 or wav audio file with facial emotion (moving mouth)
This is useful when you want to play a recorded audio speech.

### Parameters

*   `name` **[string][28]** the name of the sudio file without extention
*   `path` **[string][28]** an optinal path (leave it empty '' for QTrobot defualt audio path) (optional, default `''`)
*   `callback` **[function][29]** a callback to be called after playing the audio file (optional, default `null`)

[1]: #constructor

[2]: #parameters

[3]: #call_service

[4]: #parameters-1

[5]: #publish

[6]: #parameters-2

[7]: #subscribe

[8]: #parameters-3

[9]: #show_emotion

[10]: #parameters-4

[11]: #play_gesture

[12]: #parameters-5

[13]: #set_volume

[14]: #parameters-6

[15]: #home_motors

[16]: #parameters-7

[17]: #say_text

[18]: #parameters-8

[19]: #configure_tts

[20]: #parameters-9

[21]: #play_audio

[22]: #parameters-10

[23]: #talk_text

[24]: #parameters-11

[25]: #talk_audio

[26]: #parameters-12

[27]: https://developer.mozilla.org/docs/Web/JavaScript/Reference/Global_Objects/Object

[28]: https://developer.mozilla.org/docs/Web/JavaScript/Reference/Global_Objects/String

[29]: https://developer.mozilla.org/docs/Web/JavaScript/Reference/Statements/function

[30]: https://developer.mozilla.org/docs/Web/JavaScript/Reference/Global_Objects/Number
