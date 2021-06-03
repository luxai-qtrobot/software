/*
 * fileoverview: QTrobot SDK for Javascript using rosbridge
 * version: 1.0
 * author: ali.paikan@luxai.com (Ali Paikan)
 * license: Copyright (C) 2021 LuxAI S.A.
 */

class QTrobot {
    /**
     *  QTrobot constructor method
     *  this function will automatically call the connect method with the given 'url' to the constructor
     *  
     * @param {object} obj an object with the following fields
     *                url: the rosbridge websocket url (e.g. 'ws://127.0.0.1:9090')
     *                reconnect_time: an optionnal delay (in ms) before traying to reconnect 
     *                callback.connection: an optional callback to be called upon connection 
     *                callback.close: an optional callback to be called on closing/disconnection
     *                callback.error: an optional callback to be called upon any error on connection
     */
    constructor(obj) {
        this.url = (obj && obj.url) ? obj.url : null;
        this.reconnect_time = (obj && obj.reconnect_time) ? obj.reconnect_time : 5000;
        this.callbacks = {};
        this.callbacks.connection = (obj && obj.connection) ? obj.connection : null;
        this.callbacks.close = (obj && obj.close) ? obj.close : null;
        this.callbacks.error = (obj && obj.error) ? obj.error : null;        
        this.connect();
    }

    close() {
        this.ros = null;        
    }

    connect() {
        var self = this;
        self.ros = null;
        self.ros = new ROSLIB.Ros({ url : self.url });

        self.ros.on('connection', function() {            
            if(!self.ros) return;
            if(self.callbacks.connection) {self.callbacks.connection();}
        });

        self.ros.on('error', function(error) {
            if(self.callbacks.error) {self.callbacks.error(error);}
        });

        self.ros.on('close', function() {
            if(!self.ros) return;
            if(self.callbacks.close) {self.callbacks.close();}
            setTimeout(() => {
                self.ros.connect(self.url);
            }, self.reconnect_time);
        });
    }

    /**
     *  Call a ros service 
     * @param {string} _name service name (e.g. /qt_robot/speech/say)
     * @param {string} _type  service type (e.g.  qt_robot_interface/speech_say)
     * @param {object} _params  an object specifying the service’s params
     * @param {function} callback a callback to get the result
     */
    call_service(_name, _type, _params, callback=null) {
        var service = new ROSLIB.Service({
            ros : this.ros,
            name : _name,
            serviceType : _type
        });
        var request = new ROSLIB.ServiceRequest(_params);
        service.callService(request, function(result) {
            if(callback) { callback(result); }
        });
    }

    /**
     * Publish to a ros topic 
     * @param {string} _name topic name (e.g. /qt_robot/speech/say)
     * @param {string} _type topic message type (e.g. std_msgs/String)
     * @param {object} _params object specifying the message fileds (e.g. {data: 'hello'})
     */
    publish(_name, _type, _params) {
        var pub = new ROSLIB.Topic({
            ros : this.ros,
            name : _name,
            messageType: _type
        });
        var message = new ROSLIB.Message(_params);
        pub.publish(message);
    }

    /**
     * Subscribe to a ros topic 
     * @param {string} _name topic name (e.g. /qt_robot/joints/state)
     * @param {string} _type topic message type (e.g.  sensor_msgs/JointState)
     * @param {function} callback a callback to receive the published message 
     */
    subscribe(_name, _type, callback=null) {
        var listener = new ROSLIB.Topic({
            ros : this.ros,
            name : _name,
            messageType: _type
        });
        listener.subscribe(function(message) {
            if(callback) { callback(message); }
            listener.unsubscribe();
        });
    }

    /**
     *  Show a facial emotion on QTrobot display
     * @param {string} emotion name of the emotion (e.g. QT/happy)
     * @param {function} callback a callback to be called after playing the emotion file
     */
    show_emotion(emotion, callback=null) {
        this.call_service('/qt_robot/emotion/show', 'qt_robot_interface/emotion_show', 
        {name : emotion}, callback);
    }

    /**
     *  Play a QTrobot recorded gesture
     * @param {string} gesture name of a gesture (e.g. QT/happy)
     * @param {function} callback a callback to be called after playing the gesture
     */
    play_gesture(gesture, callback=null) {
        this.call_service('/qt_robot/gesture/play', 'qt_gesture_controller/gesture_play',
        {name : gesture}, callback);
    }

    /**
     *  Set QTrobot speaker volume 
     * @param {number} volume level in percentage (0-100)
     * @param {function} callback a callback to check the result
     */
    set_volume(volume, callback=null) {
        var volume_lookup = [0, 2, 3, 3, 4, 5, 6, 7, 10, 11, 13, 16, 20, 24, 30, 26, 44, 54, 67, 82, 100];
        var index = 0
        for (index=0; (index < volume_lookup.length) && (volume >= volume_lookup[index]); index++) { }
        var value = Math.round((index - 1) / (volume_lookup.length - 1) * 100);
        this.call_service('/qt_robot/setting/setVolume', 'qt_robot_interface/setting_setVolume',
        {volume: value}, callback);
    }

    /**
     *  Move all QTrobot motors to home position
     * @param {function} callback a callback to be called after homing all motors
     */
    home_motors(callback=null) {
        this.call_service('/qt_robot/motors/home', 'qt_motors_controller/home',
        {parts: ['head', 'right_arm', 'left_arm']}, callback);
    }

    /**
     * Speak a text using QTrobot TTS
     * @param {string} message text message 
     * @param {function} callback a callback to be called after speaking out the message
     */
    say_text(message, callback=null) {
        this.call_service('/qt_robot/speech/say',
                    'qt_robot_interface/speech_say',
                    { message : message},
                    callback);
    }

    /**
     *  Configure QTrobot TTS engine
     * @param {string} language speech langauge (e.g. en_US)
     * @param {number} pitch speech pitch value (it may not supported for all voices)
     * @param {number} speed speech talking speed (100 is the normal speed.)
     * @param {function} callback a callback to check the result
     */
    configure_tts(language='default', pitch=0, speed=0, callback=null) {        
        this.call_service('/qt_robot/speech/config', 'qt_robot_interface/speech_config',
            {language: language, pitch: pitch, speed: speed}, callback);
    }


    /**
     *  Play a mp3 or wav audio file 
     * @param {string} name the name of the sudio file without extention (e.g. QT/5LittleBunnies)
     * @param {string} path an optinal path (leave it empty '' for QTrobot defualt audio path)
     * @param {function} callback a callback to be called after playing the audio file
     */
    play_audio(name, path='', callback=null) {
        this.call_service('/qt_robot/audio/play', 'qt_robot_interface/audio_play',
        {filename: name, filepath: path}, callback);
    }


    /**
     * Speak a text using QTrobot TTS with facial emotion (moving mouth)
     * @param {string} message text message 
     * @param {function} callback a callback to be called after speaking out the message
     */
    talk_text(message, callback=null) {
         this.call_service('/qt_robot/behavior/talkText', 'qt_robot_interface/behavior_talk_text',
         { message : message}, callback);
    }

    /**
     * Play a mp3 or wav audio file with facial emotion (moving mouth)
     * This is useful when you want to play a recorded audio speech. 
     * @param {string} name the name of the sudio file without extention
     * @param {string} path an optinal path (leave it empty '' for QTrobot defualt audio path)
     * @param {function} callback a callback to be called after playing the audio file
     */
    talk_audio(name, path='', callback=null) {
         this.call_service('/qt_robot/behavior/talkAudio', 'qt_robot_interface/behavior_talk_audio',
         {filename: name, filepath: path}, callback);
    }


}
