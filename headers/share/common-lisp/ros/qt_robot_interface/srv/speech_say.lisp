; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude speech_say-request.msg.html

(cl:defclass <speech_say-request> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass speech_say-request (<speech_say-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_say-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_say-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<speech_say-request> is deprecated: use qt_robot_interface-srv:speech_say-request instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <speech_say-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:message-val is deprecated.  Use qt_robot_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_say-request>) ostream)
  "Serializes a message object of type '<speech_say-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_say-request>) istream)
  "Deserializes a message object of type '<speech_say-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_say-request>)))
  "Returns string type for a service object of type '<speech_say-request>"
  "qt_robot_interface/speech_sayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_say-request)))
  "Returns string type for a service object of type 'speech_say-request"
  "qt_robot_interface/speech_sayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_say-request>)))
  "Returns md5sum for a message object of type '<speech_say-request>"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_say-request)))
  "Returns md5sum for a message object of type 'speech_say-request"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_say-request>)))
  "Returns full string definition for message of type '<speech_say-request>"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_say-request)))
  "Returns full string definition for message of type 'speech_say-request"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_say-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_say-request>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_say-request
    (cl:cons ':message (message msg))
))
;//! \htmlinclude speech_say-response.msg.html

(cl:defclass <speech_say-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass speech_say-response (<speech_say-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_say-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_say-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<speech_say-response> is deprecated: use qt_robot_interface-srv:speech_say-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <speech_say-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_say-response>) ostream)
  "Serializes a message object of type '<speech_say-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_say-response>) istream)
  "Deserializes a message object of type '<speech_say-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_say-response>)))
  "Returns string type for a service object of type '<speech_say-response>"
  "qt_robot_interface/speech_sayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_say-response)))
  "Returns string type for a service object of type 'speech_say-response"
  "qt_robot_interface/speech_sayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_say-response>)))
  "Returns md5sum for a message object of type '<speech_say-response>"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_say-response)))
  "Returns md5sum for a message object of type 'speech_say-response"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_say-response>)))
  "Returns full string definition for message of type '<speech_say-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_say-response)))
  "Returns full string definition for message of type 'speech_say-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_say-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_say-response>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_say-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'speech_say)))
  'speech_say-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'speech_say)))
  'speech_say-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_say)))
  "Returns string type for a service object of type '<speech_say>"
  "qt_robot_interface/speech_say")