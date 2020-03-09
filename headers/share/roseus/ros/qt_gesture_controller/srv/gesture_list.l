;; Auto-generated. Do not edit!


(when (boundp 'qt_gesture_controller::gesture_list)
  (if (not (find-package "QT_GESTURE_CONTROLLER"))
    (make-package "QT_GESTURE_CONTROLLER"))
  (shadow 'gesture_list (find-package "QT_GESTURE_CONTROLLER")))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_LIST")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_LIST"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_LISTREQUEST")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_LISTREQUEST"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_LISTRESPONSE")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_LISTRESPONSE"))

(in-package "ROS")





(defclass qt_gesture_controller::gesture_listRequest
  :super ros::object
  :slots ())

(defmethod qt_gesture_controller::gesture_listRequest
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

(defclass qt_gesture_controller::gesture_listResponse
  :super ros::object
  :slots (_gestures _status ))

(defmethod qt_gesture_controller::gesture_listResponse
  (:init
   (&key
    ((:gestures __gestures) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _gestures __gestures)
   (setq _status __status)
   self)
  (:gestures
   (&optional __gestures)
   (if __gestures (setq _gestures __gestures)) _gestures)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string[] _gestures
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _gestures)) 4
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _gestures
     (write-long (length _gestures) s)
     (dolist (elem _gestures)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _gestures
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gestures (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _gestures i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_gesture_controller::gesture_list
  :super ros::object
  :slots ())

(setf (get qt_gesture_controller::gesture_list :md5sum-) "fff31f89d10c3103fdbdb3a5212feb94")
(setf (get qt_gesture_controller::gesture_list :datatype-) "qt_gesture_controller/gesture_list")
(setf (get qt_gesture_controller::gesture_list :request) qt_gesture_controller::gesture_listRequest)
(setf (get qt_gesture_controller::gesture_list :response) qt_gesture_controller::gesture_listResponse)

(defmethod qt_gesture_controller::gesture_listRequest
  (:response () (instance qt_gesture_controller::gesture_listResponse :init)))

(setf (get qt_gesture_controller::gesture_listRequest :md5sum-) "fff31f89d10c3103fdbdb3a5212feb94")
(setf (get qt_gesture_controller::gesture_listRequest :datatype-) "qt_gesture_controller/gesture_listRequest")
(setf (get qt_gesture_controller::gesture_listRequest :definition-)
      "
---
string[] gestures
bool status

")

(setf (get qt_gesture_controller::gesture_listResponse :md5sum-) "fff31f89d10c3103fdbdb3a5212feb94")
(setf (get qt_gesture_controller::gesture_listResponse :datatype-) "qt_gesture_controller/gesture_listResponse")
(setf (get qt_gesture_controller::gesture_listResponse :definition-)
      "
---
string[] gestures
bool status

")



(provide :qt_gesture_controller/gesture_list "fff31f89d10c3103fdbdb3a5212feb94")


