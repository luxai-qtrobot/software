;; Auto-generated. Do not edit!


(when (boundp 'qt_motors_controller::MotorState)
  (if (not (find-package "QT_MOTORS_CONTROLLER"))
    (make-package "QT_MOTORS_CONTROLLER"))
  (shadow 'MotorState (find-package "QT_MOTORS_CONTROLLER")))
(unless (find-package "QT_MOTORS_CONTROLLER::MOTORSTATE")
  (make-package "QT_MOTORS_CONTROLLER::MOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotorState.msg.html


(defclass qt_motors_controller::MotorState
  :super ros::object
  :slots (_name _temperature _voltage ))

(defmethod qt_motors_controller::MotorState
  (:init
   (&key
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:temperature __temperature) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:voltage __voltage) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _name __name)
   (setq _temperature __temperature)
   (setq _voltage __voltage)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:serialization-length
   ()
   (+
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float32[] _temperature
    (* 4    (length _temperature)) 4
    ;; float32[] _voltage
    (* 4    (length _voltage)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _temperature
     (write-long (length _temperature) s)
     (dotimes (i (length _temperature))
       (sys::poke (elt _temperature i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _voltage
     (write-long (length _voltage) s)
     (dotimes (i (length _voltage))
       (sys::poke (elt _voltage i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _temperature
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temperature (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _temperature i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _voltage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _voltage (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _voltage i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get qt_motors_controller::MotorState :md5sum-) "c8e09c6156fe0a406db90d8632033e1e")
(setf (get qt_motors_controller::MotorState :datatype-) "qt_motors_controller/MotorState")
(setf (get qt_motors_controller::MotorState :definition-)
      "#std_msgs/Header header
string[] name
float32[] temperature
float32[] voltage

")



(provide :qt_motors_controller/MotorState "c8e09c6156fe0a406db90d8632033e1e")


