;; Auto-generated. Do not edit!


(when (boundp 'qt_motors_controller::set_control_mode)
  (if (not (find-package "QT_MOTORS_CONTROLLER"))
    (make-package "QT_MOTORS_CONTROLLER"))
  (shadow 'set_control_mode (find-package "QT_MOTORS_CONTROLLER")))
(unless (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODE")
  (make-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODE"))
(unless (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST")
  (make-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(unless (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODERESPONSE")
  (make-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODERESPONSE"))

(in-package "ROS")





(intern "*M_ON*" (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(shadow '*M_ON* (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(defconstant qt_motors_controller::set_control_modeRequest::*M_ON* 0)
(intern "*M_OFF*" (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(shadow '*M_OFF* (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(defconstant qt_motors_controller::set_control_modeRequest::*M_OFF* 1)
(intern "*M_BREAK*" (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(shadow '*M_BREAK* (find-package "QT_MOTORS_CONTROLLER::SET_CONTROL_MODEREQUEST"))
(defconstant qt_motors_controller::set_control_modeRequest::*M_BREAK* 2)
(defclass qt_motors_controller::set_control_modeRequest
  :super ros::object
  :slots (_parts _mode ))

(defmethod qt_motors_controller::set_control_modeRequest
  (:init
   (&key
    ((:parts __parts) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _parts __parts)
   (setq _mode (round __mode))
   self)
  (:parts
   (&optional __parts)
   (if __parts (setq _parts __parts)) _parts)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; string[] _parts
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _parts)) 4
    ;; uint8 _mode
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
     ;; uint8 _mode
       (write-byte _mode s)
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
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_motors_controller::set_control_modeResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_motors_controller::set_control_modeResponse
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

(defclass qt_motors_controller::set_control_mode
  :super ros::object
  :slots ())

(setf (get qt_motors_controller::set_control_mode :md5sum-) "29ec7411588be56f1e5fa3f64ddace43")
(setf (get qt_motors_controller::set_control_mode :datatype-) "qt_motors_controller/set_control_mode")
(setf (get qt_motors_controller::set_control_mode :request) qt_motors_controller::set_control_modeRequest)
(setf (get qt_motors_controller::set_control_mode :response) qt_motors_controller::set_control_modeResponse)

(defmethod qt_motors_controller::set_control_modeRequest
  (:response () (instance qt_motors_controller::set_control_modeResponse :init)))

(setf (get qt_motors_controller::set_control_modeRequest :md5sum-) "29ec7411588be56f1e5fa3f64ddace43")
(setf (get qt_motors_controller::set_control_modeRequest :datatype-) "qt_motors_controller/set_control_modeRequest")
(setf (get qt_motors_controller::set_control_modeRequest :definition-)
      "
uint8 M_ON=0
uint8 M_OFF=1
uint8 M_BREAK=2


string[] parts
uint8  mode
---
bool status

")

(setf (get qt_motors_controller::set_control_modeResponse :md5sum-) "29ec7411588be56f1e5fa3f64ddace43")
(setf (get qt_motors_controller::set_control_modeResponse :datatype-) "qt_motors_controller/set_control_modeResponse")
(setf (get qt_motors_controller::set_control_modeResponse :definition-)
      "
uint8 M_ON=0
uint8 M_OFF=1
uint8 M_BREAK=2


string[] parts
uint8  mode
---
bool status

")



(provide :qt_motors_controller/set_control_mode "29ec7411588be56f1e5fa3f64ddace43")


