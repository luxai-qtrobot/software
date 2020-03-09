;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::behavior_talk_text)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'behavior_talk_text (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_TEXT")
  (make-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_TEXT"))
(unless (find-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_TEXTREQUEST")
  (make-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_TEXTREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_TEXTRESPONSE")
  (make-package "QT_ROBOT_INTERFACE::BEHAVIOR_TALK_TEXTRESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::behavior_talk_textRequest
  :super ros::object
  :slots (_message ))

(defmethod qt_robot_interface::behavior_talk_textRequest
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass qt_robot_interface::behavior_talk_textResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::behavior_talk_textResponse
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

(defclass qt_robot_interface::behavior_talk_text
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::behavior_talk_text :md5sum-) "27e2edee8a095bc44ea85df9f9df3f10")
(setf (get qt_robot_interface::behavior_talk_text :datatype-) "qt_robot_interface/behavior_talk_text")
(setf (get qt_robot_interface::behavior_talk_text :request) qt_robot_interface::behavior_talk_textRequest)
(setf (get qt_robot_interface::behavior_talk_text :response) qt_robot_interface::behavior_talk_textResponse)

(defmethod qt_robot_interface::behavior_talk_textRequest
  (:response () (instance qt_robot_interface::behavior_talk_textResponse :init)))

(setf (get qt_robot_interface::behavior_talk_textRequest :md5sum-) "27e2edee8a095bc44ea85df9f9df3f10")
(setf (get qt_robot_interface::behavior_talk_textRequest :datatype-) "qt_robot_interface/behavior_talk_textRequest")
(setf (get qt_robot_interface::behavior_talk_textRequest :definition-)
      "string message
---
bool status

")

(setf (get qt_robot_interface::behavior_talk_textResponse :md5sum-) "27e2edee8a095bc44ea85df9f9df3f10")
(setf (get qt_robot_interface::behavior_talk_textResponse :datatype-) "qt_robot_interface/behavior_talk_textResponse")
(setf (get qt_robot_interface::behavior_talk_textResponse :definition-)
      "string message
---
bool status

")



(provide :qt_robot_interface/behavior_talk_text "27e2edee8a095bc44ea85df9f9df3f10")


