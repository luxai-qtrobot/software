;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::emotion_stop)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'emotion_stop (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::EMOTION_STOP")
  (make-package "QT_ROBOT_INTERFACE::EMOTION_STOP"))
(unless (find-package "QT_ROBOT_INTERFACE::EMOTION_STOPREQUEST")
  (make-package "QT_ROBOT_INTERFACE::EMOTION_STOPREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::EMOTION_STOPRESPONSE")
  (make-package "QT_ROBOT_INTERFACE::EMOTION_STOPRESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::emotion_stopRequest
  :super ros::object
  :slots ())

(defmethod qt_robot_interface::emotion_stopRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass qt_robot_interface::emotion_stopResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::emotion_stopResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_robot_interface::emotion_stop
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::emotion_stop :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get qt_robot_interface::emotion_stop :datatype-) "qt_robot_interface/emotion_stop")
(setf (get qt_robot_interface::emotion_stop :request) qt_robot_interface::emotion_stopRequest)
(setf (get qt_robot_interface::emotion_stop :response) qt_robot_interface::emotion_stopResponse)

(defmethod qt_robot_interface::emotion_stopRequest
  (:response () (instance qt_robot_interface::emotion_stopResponse :init)))

(setf (get qt_robot_interface::emotion_stopRequest :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get qt_robot_interface::emotion_stopRequest :datatype-) "qt_robot_interface/emotion_stopRequest")
(setf (get qt_robot_interface::emotion_stopRequest :definition-)
      "---
bool status

")

(setf (get qt_robot_interface::emotion_stopResponse :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get qt_robot_interface::emotion_stopResponse :datatype-) "qt_robot_interface/emotion_stopResponse")
(setf (get qt_robot_interface::emotion_stopResponse :definition-)
      "---
bool status

")



(provide :qt_robot_interface/emotion_stop "3a1255d4d998bd4d6585c64639b5ee9a")


