;; Auto-generated. Do not edit!


(when (boundp 'qt_motors_controller::set_velocity)
  (if (not (find-package "QT_MOTORS_CONTROLLER"))
    (make-package "QT_MOTORS_CONTROLLER"))
  (shadow 'set_velocity (find-package "QT_MOTORS_CONTROLLER")))
(unless (find-package "QT_MOTORS_CONTROLLER::SET_VELOCITY")
  (make-package "QT_MOTORS_CONTROLLER::SET_VELOCITY"))
(unless (find-package "QT_MOTORS_CONTROLLER::SET_VELOCITYREQUEST")
  (make-package "QT_MOTORS_CONTROLLER::SET_VELOCITYREQUEST"))
(unless (find-package "QT_MOTORS_CONTROLLER::SET_VELOCITYRESPONSE")
  (make-package "QT_MOTORS_CONTROLLER::SET_VELOCITYRESPONSE"))

(in-package "ROS")





(defclass qt_motors_controller::set_velocityRequest
  :super ros::object
  :slots (_parts _velocity ))

(defmethod qt_motors_controller::set_velocityRequest
  (:init
   (&key
    ((:parts __parts) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:velocity __velocity) 0)
    )
   (send-super :init)
   (setq _parts __parts)
   (setq _velocity (round __velocity))
   self)
  (:parts
   (&optional __parts)
   (if __parts (setq _parts __parts)) _parts)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; string[] _parts
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _parts)) 4
    ;; uint8 _velocity
    1
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
     ;; uint8 _velocity
       (write-byte _velocity s)
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
   ;; uint8 _velocity
     (setq _velocity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_motors_controller::set_velocityResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_motors_controller::set_velocityResponse
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

(defclass qt_motors_controller::set_velocity
  :super ros::object
  :slots ())

(setf (get qt_motors_controller::set_velocity :md5sum-) "68003f66a5a441a7e064a7fc5cd19661")
(setf (get qt_motors_controller::set_velocity :datatype-) "qt_motors_controller/set_velocity")
(setf (get qt_motors_controller::set_velocity :request) qt_motors_controller::set_velocityRequest)
(setf (get qt_motors_controller::set_velocity :response) qt_motors_controller::set_velocityResponse)

(defmethod qt_motors_controller::set_velocityRequest
  (:response () (instance qt_motors_controller::set_velocityResponse :init)))

(setf (get qt_motors_controller::set_velocityRequest :md5sum-) "68003f66a5a441a7e064a7fc5cd19661")
(setf (get qt_motors_controller::set_velocityRequest :datatype-) "qt_motors_controller/set_velocityRequest")
(setf (get qt_motors_controller::set_velocityRequest :definition-)
      "
string[] parts
uint8  velocity
---
bool status

")

(setf (get qt_motors_controller::set_velocityResponse :md5sum-) "68003f66a5a441a7e064a7fc5cd19661")
(setf (get qt_motors_controller::set_velocityResponse :datatype-) "qt_motors_controller/set_velocityResponse")
(setf (get qt_motors_controller::set_velocityResponse :definition-)
      "
string[] parts
uint8  velocity
---
bool status

")



(provide :qt_motors_controller/set_velocity "68003f66a5a441a7e064a7fc5cd19661")


