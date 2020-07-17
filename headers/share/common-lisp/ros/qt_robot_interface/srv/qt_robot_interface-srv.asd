
(cl:in-package :asdf)

(defsystem "qt_robot_interface-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "audio_play" :depends-on ("_package_audio_play"))
    (:file "_package_audio_play" :depends-on ("_package"))
    (:file "audio_stop" :depends-on ("_package_audio_stop"))
    (:file "_package_audio_stop" :depends-on ("_package"))
    (:file "behavior_talk_audio" :depends-on ("_package_behavior_talk_audio"))
    (:file "_package_behavior_talk_audio" :depends-on ("_package"))
    (:file "behavior_talk_text" :depends-on ("_package_behavior_talk_text"))
    (:file "_package_behavior_talk_text" :depends-on ("_package"))
    (:file "emotion_show" :depends-on ("_package_emotion_show"))
    (:file "_package_emotion_show" :depends-on ("_package"))
    (:file "emotion_stop" :depends-on ("_package_emotion_stop"))
    (:file "_package_emotion_stop" :depends-on ("_package"))
    (:file "gesture_play" :depends-on ("_package_gesture_play"))
    (:file "_package_gesture_play" :depends-on ("_package"))
    (:file "setting_setVolume" :depends-on ("_package_setting_setVolume"))
    (:file "_package_setting_setVolume" :depends-on ("_package"))
    (:file "speech_config" :depends-on ("_package_speech_config"))
    (:file "_package_speech_config" :depends-on ("_package"))
    (:file "speech_say" :depends-on ("_package_speech_say"))
    (:file "_package_speech_say" :depends-on ("_package"))
    (:file "speech_stop" :depends-on ("_package_speech_stop"))
    (:file "_package_speech_stop" :depends-on ("_package"))
  ))