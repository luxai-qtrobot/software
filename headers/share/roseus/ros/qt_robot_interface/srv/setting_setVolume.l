;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::setting_setVolume)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'setting_setVolume (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::SETTING_SETVOLUME")
  (make-package "QT_ROBOT_INTERFACE::SETTING_SETVOLUME"))
(unless (find-package "QT_ROBOT_INTERFACE::SETTING_SETVOLUMEREQUEST")
  (make-package "QT_ROBOT_INTERFACE::SETTING_SETVOLUMEREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::SETTING_SETVOLUMERESPONSE")
  (make-package "QT_ROBOT_INTERFACE::SETTING_SETVOLUMERESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::setting_setVolumeRequest
  :super ros::object
  :slots (_volume ))

(defmethod qt_robot_interface::setting_setVolumeRequest
  (:init
   (&key
    ((:volume __volume) 0)
    )
   (send-super :init)
   (setq _volume (round __volume))
   self)
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:serialization-length
   ()
   (+
    ;; uint8 _volume
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _volume
       (write-byte _volume s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _volume
     (setq _volume (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_robot_interface::setting_setVolumeResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::setting_setVolumeResponse
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

(defclass qt_robot_interface::setting_setVolume
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::setting_setVolume :md5sum-) "14f0bfd003d9dc3318a211b307c7e7ab")
(setf (get qt_robot_interface::setting_setVolume :datatype-) "qt_robot_interface/setting_setVolume")
(setf (get qt_robot_interface::setting_setVolume :request) qt_robot_interface::setting_setVolumeRequest)
(setf (get qt_robot_interface::setting_setVolume :response) qt_robot_interface::setting_setVolumeResponse)

(defmethod qt_robot_interface::setting_setVolumeRequest
  (:response () (instance qt_robot_interface::setting_setVolumeResponse :init)))

(setf (get qt_robot_interface::setting_setVolumeRequest :md5sum-) "14f0bfd003d9dc3318a211b307c7e7ab")
(setf (get qt_robot_interface::setting_setVolumeRequest :datatype-) "qt_robot_interface/setting_setVolumeRequest")
(setf (get qt_robot_interface::setting_setVolumeRequest :definition-)
      "uint8 volume
---
bool status

")

(setf (get qt_robot_interface::setting_setVolumeResponse :md5sum-) "14f0bfd003d9dc3318a211b307c7e7ab")
(setf (get qt_robot_interface::setting_setVolumeResponse :datatype-) "qt_robot_interface/setting_setVolumeResponse")
(setf (get qt_robot_interface::setting_setVolumeResponse :definition-)
      "uint8 volume
---
bool status

")



(provide :qt_robot_interface/setting_setVolume "14f0bfd003d9dc3318a211b307c7e7ab")


