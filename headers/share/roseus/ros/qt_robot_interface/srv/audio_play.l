;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::audio_play)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'audio_play (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::AUDIO_PLAY")
  (make-package "QT_ROBOT_INTERFACE::AUDIO_PLAY"))
(unless (find-package "QT_ROBOT_INTERFACE::AUDIO_PLAYREQUEST")
  (make-package "QT_ROBOT_INTERFACE::AUDIO_PLAYREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::AUDIO_PLAYRESPONSE")
  (make-package "QT_ROBOT_INTERFACE::AUDIO_PLAYRESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::audio_playRequest
  :super ros::object
  :slots (_filename _filepath ))

(defmethod qt_robot_interface::audio_playRequest
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

(defclass qt_robot_interface::audio_playResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::audio_playResponse
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

(defclass qt_robot_interface::audio_play
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::audio_play :md5sum-) "52268a262dc8cf03d5103995c9714ffa")
(setf (get qt_robot_interface::audio_play :datatype-) "qt_robot_interface/audio_play")
(setf (get qt_robot_interface::audio_play :request) qt_robot_interface::audio_playRequest)
(setf (get qt_robot_interface::audio_play :response) qt_robot_interface::audio_playResponse)

(defmethod qt_robot_interface::audio_playRequest
  (:response () (instance qt_robot_interface::audio_playResponse :init)))

(setf (get qt_robot_interface::audio_playRequest :md5sum-) "52268a262dc8cf03d5103995c9714ffa")
(setf (get qt_robot_interface::audio_playRequest :datatype-) "qt_robot_interface/audio_playRequest")
(setf (get qt_robot_interface::audio_playRequest :definition-)
      "string filename
string filepath
---
bool status

")

(setf (get qt_robot_interface::audio_playResponse :md5sum-) "52268a262dc8cf03d5103995c9714ffa")
(setf (get qt_robot_interface::audio_playResponse :datatype-) "qt_robot_interface/audio_playResponse")
(setf (get qt_robot_interface::audio_playResponse :definition-)
      "string filename
string filepath
---
bool status

")



(provide :qt_robot_interface/audio_play "52268a262dc8cf03d5103995c9714ffa")


