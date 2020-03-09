;; Auto-generated. Do not edit!


(when (boundp 'qt_gesture_controller::gesture_record)
  (if (not (find-package "QT_GESTURE_CONTROLLER"))
    (make-package "QT_GESTURE_CONTROLLER"))
  (shadow 'gesture_record (find-package "QT_GESTURE_CONTROLLER")))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_RECORD")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_RECORD"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_RECORDREQUEST")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_RECORDREQUEST"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_RECORDRESPONSE")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_RECORDRESPONSE"))

(in-package "ROS")





(defclass qt_gesture_controller::gesture_recordRequest
  :super ros::object
  :slots (_parts _idleParts _wait _timeout ))

(defmethod qt_gesture_controller::gesture_recordRequest
  (:init
   (&key
    ((:parts __parts) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:idleParts __idleParts) nil)
    ((:wait __wait) 0)
    ((:timeout __timeout) 0)
    )
   (send-super :init)
   (setq _parts __parts)
   (setq _idleParts __idleParts)
   (setq _wait (round __wait))
   (setq _timeout (round __timeout))
   self)
  (:parts
   (&optional __parts)
   (if __parts (setq _parts __parts)) _parts)
  (:idleParts
   (&optional __idleParts)
   (if __idleParts (setq _idleParts __idleParts)) _idleParts)
  (:wait
   (&optional __wait)
   (if __wait (setq _wait __wait)) _wait)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:serialization-length
   ()
   (+
    ;; string[] _parts
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _parts)) 4
    ;; bool _idleParts
    1
    ;; uint8 _wait
    1
    ;; uint8 _timeout
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
     ;; bool _idleParts
       (if _idleParts (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _wait
       (write-byte _wait s)
     ;; uint8 _timeout
       (write-byte _timeout s)
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
   ;; bool _idleParts
     (setq _idleParts (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _wait
     (setq _wait (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _timeout
     (setq _timeout (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass qt_gesture_controller::gesture_recordResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_gesture_controller::gesture_recordResponse
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

(defclass qt_gesture_controller::gesture_record
  :super ros::object
  :slots ())

(setf (get qt_gesture_controller::gesture_record :md5sum-) "f920c6a17216e8fe1e02e50dada8c9d0")
(setf (get qt_gesture_controller::gesture_record :datatype-) "qt_gesture_controller/gesture_record")
(setf (get qt_gesture_controller::gesture_record :request) qt_gesture_controller::gesture_recordRequest)
(setf (get qt_gesture_controller::gesture_record :response) qt_gesture_controller::gesture_recordResponse)

(defmethod qt_gesture_controller::gesture_recordRequest
  (:response () (instance qt_gesture_controller::gesture_recordResponse :init)))

(setf (get qt_gesture_controller::gesture_recordRequest :md5sum-) "f920c6a17216e8fe1e02e50dada8c9d0")
(setf (get qt_gesture_controller::gesture_recordRequest :datatype-) "qt_gesture_controller/gesture_recordRequest")
(setf (get qt_gesture_controller::gesture_recordRequest :definition-)
      "
string[] parts


bool idleParts


uint8 wait


uint8 timeout

---
bool status

")

(setf (get qt_gesture_controller::gesture_recordResponse :md5sum-) "f920c6a17216e8fe1e02e50dada8c9d0")
(setf (get qt_gesture_controller::gesture_recordResponse :datatype-) "qt_gesture_controller/gesture_recordResponse")
(setf (get qt_gesture_controller::gesture_recordResponse :definition-)
      "
string[] parts


bool idleParts


uint8 wait


uint8 timeout

---
bool status

")



(provide :qt_gesture_controller/gesture_record "f920c6a17216e8fe1e02e50dada8c9d0")


