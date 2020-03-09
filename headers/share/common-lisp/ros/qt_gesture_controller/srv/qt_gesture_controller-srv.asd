
(cl:in-package :asdf)

(defsystem "qt_gesture_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gesture_list" :depends-on ("_package_gesture_list"))
    (:file "_package_gesture_list" :depends-on ("_package"))
    (:file "gesture_play" :depends-on ("_package_gesture_play"))
    (:file "_package_gesture_play" :depends-on ("_package"))
    (:file "gesture_record" :depends-on ("_package_gesture_record"))
    (:file "_package_gesture_record" :depends-on ("_package"))
    (:file "gesture_save" :depends-on ("_package_gesture_save"))
    (:file "_package_gesture_save" :depends-on ("_package"))
    (:file "gesture_stop" :depends-on ("_package_gesture_stop"))
    (:file "_package_gesture_stop" :depends-on ("_package"))
  ))