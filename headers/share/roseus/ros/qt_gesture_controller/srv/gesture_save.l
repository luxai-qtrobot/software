;; Auto-generated. Do not edit!


(when (boundp 'qt_gesture_controller::gesture_save)
  (if (not (find-package "QT_GESTURE_CONTROLLER"))
    (make-package "QT_GESTURE_CONTROLLER"))
  (shadow 'gesture_save (find-package "QT_GESTURE_CONTROLLER")))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_SAVE")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_SAVE"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_SAVEREQUEST")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_SAVEREQUEST"))
(unless (find-package "QT_GESTURE_CONTROLLER::GESTURE_SAVERESPONSE")
  (make-package "QT_GESTURE_CONTROLLER::GESTURE_SAVERESPONSE"))

(in-package "ROS")





(defclass qt_gesture_controller::gesture_saveRequest
  :super ros::object
  :slots (_name _path ))

(defmethod qt_gesture_controller::gesture_saveRequest
  (:init
   (&key
    ((:name __name) "")
    ((:path __path) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _path (string __path))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _path
    4 (length _path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _path
       (write-long (length _path) s) (princ _path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass qt_gesture_controller::gesture_saveResponse
  :super ros::object
  :slots (_status ))

(defmethod qt_gesture_controller::gesture_saveResponse
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

(defclass qt_gesture_controller::gesture_save
  :super ros::object
  :slots ())

(setf (get qt_gesture_controller::gesture_save :md5sum-) "088f4facf8938c7925f9bdd7cacce95b")
(setf (get qt_gesture_controller::gesture_save :datatype-) "qt_gesture_controller/gesture_save")
(setf (get qt_gesture_controller::gesture_save :request) qt_gesture_controller::gesture_saveRequest)
(setf (get qt_gesture_controller::gesture_save :response) qt_gesture_controller::gesture_saveResponse)

(defmethod qt_gesture_controller::gesture_saveRequest
  (:response () (instance qt_gesture_controller::gesture_saveResponse :init)))

(setf (get qt_gesture_controller::gesture_saveRequest :md5sum-) "088f4facf8938c7925f9bdd7cacce95b")
(setf (get qt_gesture_controller::gesture_saveRequest :datatype-) "qt_gesture_controller/gesture_saveRequest")
(setf (get qt_gesture_controller::gesture_saveRequest :definition-)
      "
string name


string path

---
bool status

")

(setf (get qt_gesture_controller::gesture_saveResponse :md5sum-) "088f4facf8938c7925f9bdd7cacce95b")
(setf (get qt_gesture_controller::gesture_saveResponse :datatype-) "qt_gesture_controller/gesture_saveResponse")
(setf (get qt_gesture_controller::gesture_saveResponse :definition-)
      "
string name


string path

---
bool status

")



(provide :qt_gesture_controller/gesture_save "088f4facf8938c7925f9bdd7cacce95b")


