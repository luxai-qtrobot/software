;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::speech_config)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'speech_config (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::SPEECH_CONFIG")
  (make-package "QT_ROBOT_INTERFACE::SPEECH_CONFIG"))
(unless (find-package "QT_ROBOT_INTERFACE::SPEECH_CONFIGREQUEST")
  (make-package "QT_ROBOT_INTERFACE::SPEECH_CONFIGREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::SPEECH_CONFIGRESPONSE")
  (make-package "QT_ROBOT_INTERFACE::SPEECH_CONFIGRESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::speech_configRequest
  :super ros::object
  :slots (_language _pitch _speed ))

(defmethod qt_robot_interface::speech_configRequest
  (:init
   (&key
    ((:language __language) "")
    ((:pitch __pitch) 0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _language (string __language))
   (setq _pitch (round __pitch))
   (setq _speed (round __speed))
   self)
  (:language
   (&optional __language)
   (if __language (setq _language __language)) _language)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; string _language
    4 (length _language)
    ;; int16 _pitch
    2
    ;; int16 _speed
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _language
       (write-long (length _language) s) (princ _language s)
     ;; int16 _pitch
       (write-word _pitch s)
     ;; int16 _speed
       (write-word _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _language
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _language (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _pitch
     (setq _pitch (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _speed
     (setq _speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass qt_robot_interface::speech_configResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::speech_configResponse
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

(defclass qt_robot_interface::speech_config
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::speech_config :md5sum-) "bb0832ad049396f7ac17944e2242235c")
(setf (get qt_robot_interface::speech_config :datatype-) "qt_robot_interface/speech_config")
(setf (get qt_robot_interface::speech_config :request) qt_robot_interface::speech_configRequest)
(setf (get qt_robot_interface::speech_config :response) qt_robot_interface::speech_configResponse)

(defmethod qt_robot_interface::speech_configRequest
  (:response () (instance qt_robot_interface::speech_configResponse :init)))

(setf (get qt_robot_interface::speech_configRequest :md5sum-) "bb0832ad049396f7ac17944e2242235c")
(setf (get qt_robot_interface::speech_configRequest :datatype-) "qt_robot_interface/speech_configRequest")
(setf (get qt_robot_interface::speech_configRequest :definition-)
      "string language
int16  pitch
int16  speed
---
bool status

")

(setf (get qt_robot_interface::speech_configResponse :md5sum-) "bb0832ad049396f7ac17944e2242235c")
(setf (get qt_robot_interface::speech_configResponse :datatype-) "qt_robot_interface/speech_configResponse")
(setf (get qt_robot_interface::speech_configResponse :definition-)
      "string language
int16  pitch
int16  speed
---
bool status

")



(provide :qt_robot_interface/speech_config "bb0832ad049396f7ac17944e2242235c")


