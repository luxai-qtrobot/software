;; Auto-generated. Do not edit!


(when (boundp 'qt_motors_controller::home)
  (if (not (find-package "QT_MOTORS_CONTROLLER"))
    (make-package "QT_MOTORS_CONTROLLER"))
  (shadow 'home (find-package "QT_MOTORS_CONTROLLER")))
(unless (find-package "QT_MOTORS_CONTROLLER::HOME")
  (make-package "QT_MOTORS_CONTROLLER::HOME"))
(unless (find-package "QT_MOTORS_CONTROLLER::HOMEREQUEST")
  (make-package "QT_MOTORS_CONTROLLER::HOMEREQUEST"))
(unless (find-package "QT_MOTORS_CONTROLLER::HOMERESPONSE")
  (make-package "QT_MOTORS_CONTROLLER::HOMERESPONSE"))

(in-package "ROS")





(defclass qt_motors_controller::homeRequest
  :super ros::object
  :slots (_parts ))

(defmethod qt_motors_controller::homeRequest
  (:init
   (&key
    ((:parts __parts) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _parts __parts)
   self)
  (:parts
   (&optional __parts)
   (if __parts (setq _parts __parts)) _parts)
  (:serialization-length
   ()
   (+
    ;; string[] _parts
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _parts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _parts
     (write-long (length _parts) s)
     (dolist (elem _parts)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _parts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _parts (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _parts i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass qt_motors_controller::homeResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_motors_controller::homeResponse
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

(defclass qt_motors_controller::home
  :super ros::object
  :slots ())

(setf (get qt_motors_controller::home :md5sum-) "018aa38faa01fecffa7a4eb6cad48ceb")
(setf (get qt_motors_controller::home :datatype-) "qt_motors_controller/home")
(setf (get qt_motors_controller::home :request) qt_motors_controller::homeRequest)
(setf (get qt_motors_controller::home :response) qt_motors_controller::homeResponse)

(defmethod qt_motors_controller::homeRequest
  (:response () (instance qt_motors_controller::homeResponse :init)))

(setf (get qt_motors_controller::homeRequest :md5sum-) "018aa38faa01fecffa7a4eb6cad48ceb")
(setf (get qt_motors_controller::homeRequest :datatype-) "qt_motors_controller/homeRequest")
(setf (get qt_motors_controller::homeRequest :definition-)
      "
string[] parts
---
bool status

")

(setf (get qt_motors_controller::homeResponse :md5sum-) "018aa38faa01fecffa7a4eb6cad48ceb")
(setf (get qt_motors_controller::homeResponse :datatype-) "qt_motors_controller/homeResponse")
(setf (get qt_motors_controller::homeResponse :definition-)
      "
string[] parts
---
bool status

")



(provide :qt_motors_controller/home "018aa38faa01fecffa7a4eb6cad48ceb")


