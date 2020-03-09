; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude behavior_talk_text-request.msg.html

(cl:defclass <behavior_talk_text-request> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass behavior_talk_text-request (<behavior_talk_text-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <behavior_talk_text-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'behavior_talk_text-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<behavior_talk_text-request> is deprecated: use qt_robot_interface-srv:behavior_talk_text-request instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <behavior_talk_text-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:message-val is deprecated.  Use qt_robot_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <behavior_talk_text-request>) ostream)
  "Serializes a message object of type '<behavior_talk_text-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <behavior_talk_text-request>) istream)
  "Deserializes a message object of type '<behavior_talk_text-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<behavior_talk_text-request>)))
  "Returns string type for a service object of type '<behavior_talk_text-request>"
  "qt_robot_interface/behavior_talk_textRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'behavior_talk_text-request)))
  "Returns string type for a service object of type 'behavior_talk_text-request"
  "qt_robot_interface/behavior_talk_textRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<behavior_talk_text-request>)))
  "Returns md5sum for a message object of type '<behavior_talk_text-request>"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'behavior_talk_text-request)))
  "Returns md5sum for a message object of type 'behavior_talk_text-request"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<behavior_talk_text-request>)))
  "Returns full string definition for message of type '<behavior_talk_text-request>"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'behavior_talk_text-request)))
  "Returns full string definition for message of type 'behavior_talk_text-request"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <behavior_talk_text-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <behavior_talk_text-request>))
  "Converts a ROS message object to a list"
  (cl:list 'behavior_talk_text-request
    (cl:cons ':message (message msg))
))
;//! \htmlinclude behavior_talk_text-response.msg.html

(cl:defclass <behavior_talk_text-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass behavior_talk_text-response (<behavior_talk_text-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <behavior_talk_text-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'behavior_talk_text-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<behavior_talk_text-response> is deprecated: use qt_robot_interface-srv:behavior_talk_text-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <behavior_talk_text-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <behavior_talk_text-response>) ostream)
  "Serializes a message object of type '<behavior_talk_text-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <behavior_talk_text-response>) istream)
  "Deserializes a message object of type '<behavior_talk_text-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<behavior_talk_text-response>)))
  "Returns string type for a service object of type '<behavior_talk_text-response>"
  "qt_robot_interface/behavior_talk_textResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'behavior_talk_text-response)))
  "Returns string type for a service object of type 'behavior_talk_text-response"
  "qt_robot_interface/behavior_talk_textResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<behavior_talk_text-response>)))
  "Returns md5sum for a message object of type '<behavior_talk_text-response>"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'behavior_talk_text-response)))
  "Returns md5sum for a message object of type 'behavior_talk_text-response"
  "27e2edee8a095bc44ea85df9f9df3f10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<behavior_talk_text-response>)))
  "Returns full string definition for message of type '<behavior_talk_text-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'behavior_talk_text-response)))
  "Returns full string definition for message of type 'behavior_talk_text-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <behavior_talk_text-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <behavior_talk_text-response>))
  "Converts a ROS message object to a list"
  (cl:list 'behavior_talk_text-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'behavior_talk_text)))
  'behavior_talk_text-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'behavior_talk_text)))
  'behavior_talk_text-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'behavior_talk_text)))
  "Returns string type for a service object of type '<behavior_talk_text>"
  "qt_robot_interface/behavior_talk_text")