;; Auto-generated. Do not edit!


(when (boundp 'qt_gesture_controller::gesture_play)
  (if (not (find-package "QT_GESTURE_CONTROLLER"))
    (make-package "QT_GESTURE_CONTROLLER"))
  (shadow 'gesture_play (find-package "QT_GESTURE_CONTROLLER")))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_PLAY")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_PLAY"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_PLAYREQUEST")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_PLAYREQUEST"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_PLAYRESPONSE")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_PLAYRESPONSE"))

(in-package "ROS")





(defclass qt_gesture_controller::gesture_playRequest
  :super ros::object
  :slots (_name _speed ))

(defmethod qt_gesture_controller::gesture_playRequest
  (:init
   (&key
    ((:name __name) "")
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _speed (float __speed))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qt_gesture_controller::gesture_playResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_gesture_controller::gesture_playResponse
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

(defclass qt_gesture_controller::gesture_play
  :super ros::object
  :slots ())

(setf (get qt_gesture_controller::gesture_play :md5sum-) "b8a4b9991efb6139d3fd7428a2ee4889")
(setf (get qt_gesture_controller::gesture_play :datatype-) "qt_gesture_controller/gesture_play")
(setf (get qt_gesture_controller::gesture_play :request) qt_gesture_controller::gesture_playRequest)
(setf (get qt_gesture_controller::gesture_play :response) qt_gesture_controller::gesture_playResponse)

(defmethod qt_gesture_controller::gesture_playRequest
  (:response () (instance qt_gesture_controller::gesture_playResponse :init)))

(setf (get qt_gesture_controller::gesture_playRequest :md5sum-) "b8a4b9991efb6139d3fd7428a2ee4889")
(setf (get qt_gesture_controller::gesture_playRequest :datatype-) "qt_gesture_controller/gesture_playRequest")
(setf (get qt_gesture_controller::gesture_playRequest :definition-)
      "string name
float32 speed
---
bool status

")

(setf (get qt_gesture_controller::gesture_playResponse :md5sum-) "b8a4b9991efb6139d3fd7428a2ee4889")
(setf (get qt_gesture_controller::gesture_playResponse :datatype-) "qt_gesture_controller/gesture_playResponse")
(setf (get qt_gesture_controller::gesture_playResponse :definition-)
      "string name
float32 speed
---
bool status

")



(provide :qt_gesture_controller/gesture_play "b8a4b9991efb6139d3fd7428a2ee4889")


