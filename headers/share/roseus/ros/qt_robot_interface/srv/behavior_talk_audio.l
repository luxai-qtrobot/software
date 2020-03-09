;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::behavior_talk_audio)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'behavior_talk_audio (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_AUDIO")
  (make-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_AUDIO"))
(unless (find-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_AUDIOREQUEST")
  (make-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_AUDIOREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_AUDIORESPONSE")
  (make-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_AUDIORESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::behavior_talk_audioRequest
  :super ros::object
  :slots (_filename _filepath ))

(defmethod qt_robot_interface::behavior_talk_audioRequest
  (:init
   (&key
    ((:filename __filename) "")
    ((:filepath __filepath) "")
    )
   (send-super :init)
   (setq _filename (string __filename))
   (setq _filepath (string __filepath))
   self)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:filepath
   (&optional __filepath)
   (if __filepath (setq _filepath __filepath)) _filepath)
  (:serialization-length
   ()
   (+
    ;; string _filename
    4 (length _filename)
    ;; string _filepath
    4 (length _filepath)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;; string _filepath
       (write-long (length _filepath) s) (princ _filepath s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _filepath
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filepath (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass qt_robot_interface::behavior_talk_audioResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::behavior_talk_audioResponse
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

(defclass qt_robot_interface::behavior_talk_audio
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::behavior_talk_audio :md5sum-) "52268a262dc8cf03d5103995c9714ffa")
(setf (get qt_robot_interface::behavior_talk_audio :datatype-) "qt_robot_interface/behavior_talk_audio")
(setf (get qt_robot_interface::behavior_talk_audio :request) qt_robot_interface::behavior_talk_audioRequest)
(setf (get qt_robot_interface::behavior_talk_audio :response) qt_robot_interface::behavior_talk_audioResponse)

(defmethod qt_robot_interface::behavior_talk_audioRequest
  (:response () (instance qt_robot_interface::behavior_talk_audioResponse :init)))

(setf (get qt_robot_interface::behavior_talk_audioRequest :md5sum-) "52268a262dc8cf03d5103995c9714ffa")
(setf (get qt_robot_interface::behavior_talk_audioRequest :datatype-) "qt_robot_interface/behavior_talk_audioRequest")
(setf (get qt_robot_interface::behavior_talk_audioRequest :definition-)
      "string filename
string filepath
---
bool status

")

(setf (get qt_robot_interface::behavior_talk_audioResponse :md5sum-) "52268a262dc8cf03d5103995c9714ffa")
(setf (get qt_robot_interface::behavior_talk_audioResponse :datatype-) "qt_robot_interface/behavior_talk_audioResponse")
(setf (get qt_robot_interface::behavior_talk_audioResponse :definition-)
      "string filename
string filepath
---
bool status

")



(provide :qt_robot_interface/behavior_talk_audio "52268a262dc8cf03d5103995c9714ffa")


