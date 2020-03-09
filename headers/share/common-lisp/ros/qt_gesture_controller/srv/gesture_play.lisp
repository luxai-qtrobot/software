; Auto-generated. Do not edit!


(cl:in-package qt_gesture_controller-srv)


;//! \htmlinclude gesture_play-request.msg.html

(cl:defclass <gesture_play-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass gesture_play-request (<gesture_play-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_play-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_play-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_play-request> is deprecated: use qt_gesture_controller-srv:gesture_play-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <gesture_play-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:name-val is deprecated.  Use qt_gesture_controller-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <gesture_play-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:speed-val is deprecated.  Use qt_gesture_controller-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_play-request>) ostream)
  "Serializes a message object of type '<gesture_play-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_play-request>) istream)
  "Deserializes a message object of type '<gesture_play-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_play-request>)))
  "Returns string type for a service object of type '<gesture_play-request>"
  "qt_gesture_controller/gesture_playRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_play-request)))
  "Returns string type for a service object of type 'gesture_play-request"
  "qt_gesture_controller/gesture_playRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_play-request>)))
  "Returns md5sum for a message object of type '<gesture_play-request>"
  "b8a4b9991efb6139d3fd7428a2ee4889")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_play-request)))
  "Returns md5sum for a message object of type 'gesture_play-request"
  "b8a4b9991efb6139d3fd7428a2ee4889")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_play-request>)))
  "Returns full string definition for message of type '<gesture_play-request>"
  (cl:format cl:nil "string name~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_play-request)))
  "Returns full string definition for message of type 'gesture_play-request"
  (cl:format cl:nil "string name~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_play-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_play-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_play-request
    (cl:cons ':name (name msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude gesture_play-response.msg.html

(cl:defclass <gesture_play-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gesture_play-response (<gesture_play-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_play-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_play-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_play-response> is deprecated: use qt_gesture_controller-srv:gesture_play-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <gesture_play-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:status-val is deprecated.  Use qt_gesture_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_play-response>) ostream)
  "Serializes a message object of type '<gesture_play-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_play-response>) istream)
  "Deserializes a message object of type '<gesture_play-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_play-response>)))
  "Returns string type for a service object of type '<gesture_play-response>"
  "qt_gesture_controller/gesture_playResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_play-response)))
  "Returns string type for a service object of type 'gesture_play-response"
  "qt_gesture_controller/gesture_playResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_play-response>)))
  "Returns md5sum for a message object of type '<gesture_play-response>"
  "b8a4b9991efb6139d3fd7428a2ee4889")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_play-response)))
  "Returns md5sum for a message object of type 'gesture_play-response"
  "b8a4b9991efb6139d3fd7428a2ee4889")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_play-response>)))
  "Returns full string definition for message of type '<gesture_play-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_play-response)))
  "Returns full string definition for message of type 'gesture_play-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_play-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_play-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_play-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gesture_play)))
  'gesture_play-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gesture_play)))
  'gesture_play-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_play)))
  "Returns string type for a service object of type '<gesture_play>"
  "qt_gesture_controller/gesture_play")