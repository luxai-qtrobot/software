;; Auto-generated. Do not edit!


(when (boundp 'qt_gesture_controller::gesture_stop)
  (if (not (find-package "QT_GESTURE_CONTROLLER"))
    (make-package "QT_GESTURE_CONTROLLER"))
  (shadow 'gesture_stop (find-package "QT_GESTURE_CONTROLLER")))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_STOP")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_STOP"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_STOPREQUEST")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_STOPREQUEST"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_STOPRESPONSE")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_STOPRESPONSE"))

(in-package "ROS")





(defclass qt_gesture_controller::gesture_stopRequest
  :super ros::object
  :slots ())

(defmethod qt_gesture_controller::gesture_stopRequest
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

(defclass qt_gesture_controller::gesture_stopResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_gesture_controller::gesture_stopResponse
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

(defclass qt_gesture_controller::gesture_stop
  :super ros::object
  :slots ())

(setf (get qt_gesture_controller::gesture_stop :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get qt_gesture_controller::gesture_stop :datatype-) "qt_gesture_controller/gesture_stop")
(setf (get qt_gesture_controller::gesture_stop :request) qt_gesture_controller::gesture_stopRequest)
(setf (get qt_gesture_controller::gesture_stop :response) qt_gesture_controller::gesture_stopResponse)

(defmethod qt_gesture_controller::gesture_stopRequest
  (:response () (instance qt_gesture_controller::gesture_stopResponse :init)))

(setf (get qt_gesture_controller::gesture_stopRequest :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get qt_gesture_controller::gesture_stopRequest :datatype-) "qt_gesture_controller/gesture_stopRequest")
(setf (get qt_gesture_controller::gesture_stopRequest :definition-)
      "---
bool status

")

(setf (get qt_gesture_controller::gesture_stopResponse :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get qt_gesture_controller::gesture_stopResponse :datatype-) "qt_gesture_controller/gesture_stopResponse")
(setf (get qt_gesture_controller::gesture_stopResponse :definition-)
      "---
bool status

")



(provide :qt_gesture_controller/gesture_stop "3a1255d4d998bd4d6585c64639b5ee9a")


