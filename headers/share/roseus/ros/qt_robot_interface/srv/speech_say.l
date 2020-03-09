;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::speech_say)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'speech_say (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::SPEECH_SAY")
  (make-package "QT_ROBOT_INTERFACE::SPEECH_SAY"))
(unless (find-package "QT_ROBOT_INTERFACE::SPEECH_SAYREQUEST")
  (make-package "QT_ROBOT_INTERFACE::SPEECH_SAYREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::SPEECH_SAYRESPONSE")
  (make-package "QT_ROBOT_INTERFACE::SPEECH_SAYRESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::speech_sayRequest
  :super ros::object
  :slots (_message ))

(defmethod qt_robot_interface::speech_sayRequest
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

(defclass qt_robot_interface::speech_sayResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::speech_sayResponse
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

(defclass qt_robot_interface::speech_say
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::speech_say :md5sum-) "27e2edee8a095bc44ea85df9f9df3f10")
(setf (get qt_robot_interface::speech_say :datatype-) "qt_robot_interface/speech_say")
(setf (get qt_robot_interface::speech_say :request) qt_robot_interface::speech_sayRequest)
(setf (get qt_robot_interface::speech_say :response) qt_robot_interface::speech_sayResponse)

(defmethod qt_robot_interface::speech_sayRequest
  (:response () (instance qt_robot_interface::speech_sayResponse :init)))

(setf (get qt_robot_interface::speech_sayRequest :md5sum-) "27e2edee8a095bc44ea85df9f9df3f10")
(setf (get qt_robot_interface::speech_sayRequest :datatype-) "qt_robot_interface/speech_sayRequest")
(setf (get qt_robot_interface::speech_sayRequest :definition-)
      "string message
---
bool status

")

(setf (get qt_robot_interface::speech_sayResponse :md5sum-) "27e2edee8a095bc44ea85df9f9df3f10")
(setf (get qt_robot_interface::speech_sayResponse :datatype-) "qt_robot_interface/speech_sayResponse")
(setf (get qt_robot_interface::speech_sayResponse :definition-)
      "string message
---
bool status

")



(provide :qt_robot_interface/speech_say "27e2edee8a095bc44ea85df9f9df3f10")


