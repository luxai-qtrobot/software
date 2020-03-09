;; Auto-generated. Do not edit!


(when (boundp 'qt_robot_interface::emotion_show)
  (if (not (find-package "QT_ROBOT_INTERFACE"))
    (make-package "QT_ROBOT_INTERFACE"))
  (shadow 'emotion_show (find-package "QT_ROBOT_INTERFACE")))
(unless (find-package "QT_ROBOT_INTERFACE::EMOTION_SHOW")
  (make-package "QT_ROBOT_INTERFACE::EMOTION_SHOW"))
(unless (find-package "QT_ROBOT_INTERFACE::EMOTION_SHOWREQUEST")
  (make-package "QT_ROBOT_INTERFACE::EMOTION_SHOWREQUEST"))
(unless (find-package "QT_ROBOT_INTERFACE::EMOTION_SHOWRESPONSE")
  (make-package "QT_ROBOT_INTERFACE::EMOTION_SHOWRESPONSE"))

(in-package "ROS")





(defclass qt_robot_interface::emotion_showRequest
  :super ros::object
  :slots (_name ))

(defmethod qt_robot_interface::emotion_showRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass qt_robot_interface::emotion_showResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_robot_interface::emotion_showResponse
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

(defclass qt_robot_interface::emotion_show
  :super ros::object
  :slots ())

(setf (get qt_robot_interface::emotion_show :md5sum-) "186befe2a32d448f6a8e15271a5e2624")
(setf (get qt_robot_interface::emotion_show :datatype-) "qt_robot_interface/emotion_show")
(setf (get qt_robot_interface::emotion_show :request) qt_robot_interface::emotion_showRequest)
(setf (get qt_robot_interface::emotion_show :response) qt_robot_interface::emotion_showResponse)

(defmethod qt_robot_interface::emotion_showRequest
  (:response () (instance qt_robot_interface::emotion_showResponse :init)))

(setf (get qt_robot_interface::emotion_showRequest :md5sum-) "186befe2a32d448f6a8e15271a5e2624")
(setf (get qt_robot_interface::emotion_showRequest :datatype-) "qt_robot_interface/emotion_showRequest")
(setf (get qt_robot_interface::emotion_showRequest :definition-)
      "string name
---
bool status

")

(setf (get qt_robot_interface::emotion_showResponse :md5sum-) "186befe2a32d448f6a8e15271a5e2624")
(setf (get qt_robot_interface::emotion_showResponse :datatype-) "qt_robot_interface/emotion_showResponse")
(setf (get qt_robot_interface::emotion_showResponse :definition-)
      "string name
---
bool status

")



(provide :qt_robot_interface/emotion_show "186befe2a32d448f6a8e15271a5e2624")


