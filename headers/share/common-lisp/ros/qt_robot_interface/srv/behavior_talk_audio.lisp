; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude behavior_talk_audio-request.msg.html

(cl:defclass <behavior_talk_audio-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (filepath
    :reader filepath
    :initarg :filepath
    :type cl:string
    :initform ""))
)

(cl:defclass behavior_talk_audio-request (<behavior_talk_audio-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <behavior_talk_audio-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'behavior_talk_audio-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<behavior_talk_audio-request> is deprecated: use qt_robot_interface-srv:behavior_talk_audio-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <behavior_talk_audio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:filename-val is deprecated.  Use qt_robot_interface-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'filepath-val :lambda-list '(m))
(cl:defmethod filepath-val ((m <behavior_talk_audio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:filepath-val is deprecated.  Use qt_robot_interface-srv:filepath instead.")
  (filepath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <behavior_talk_audio-request>) ostream)
  "Serializes a message object of type '<behavior_talk_audio-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filepath))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filepath))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <behavior_talk_audio-request>) istream)
  "Deserializes a message object of type '<behavior_talk_audio-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filepath) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filepath) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<behavior_talk_audio-request>)))
  "Returns string type for a service object of type '<behavior_talk_audio-request>"
  "qt_robot_interface/behavior_talk_audioRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'behavior_talk_audio-request)))
  "Returns string type for a service object of type 'behavior_talk_audio-request"
  "qt_robot_interface/behavior_talk_audioRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<behavior_talk_audio-request>)))
  "Returns md5sum for a message object of type '<behavior_talk_audio-request>"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'behavior_talk_audio-request)))
  "Returns md5sum for a message object of type 'behavior_talk_audio-request"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<behavior_talk_audio-request>)))
  "Returns full string definition for message of type '<behavior_talk_audio-request>"
  (cl:format cl:nil "string filename~%string filepath~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'behavior_talk_audio-request)))
  "Returns full string definition for message of type 'behavior_talk_audio-request"
  (cl:format cl:nil "string filename~%string filepath~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <behavior_talk_audio-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'filepath))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <behavior_talk_audio-request>))
  "Converts a ROS message object to a list"
  (cl:list 'behavior_talk_audio-request
    (cl:cons ':filename (filename msg))
    (cl:cons ':filepath (filepath msg))
))
;//! \htmlinclude behavior_talk_audio-response.msg.html

(cl:defclass <behavior_talk_audio-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass behavior_talk_audio-response (<behavior_talk_audio-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <behavior_talk_audio-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'behavior_talk_audio-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<behavior_talk_audio-response> is deprecated: use qt_robot_interface-srv:behavior_talk_audio-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <behavior_talk_audio-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <behavior_talk_audio-response>) ostream)
  "Serializes a message object of type '<behavior_talk_audio-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <behavior_talk_audio-response>) istream)
  "Deserializes a message object of type '<behavior_talk_audio-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<behavior_talk_audio-response>)))
  "Returns string type for a service object of type '<behavior_talk_audio-response>"
  "qt_robot_interface/behavior_talk_audioResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'behavior_talk_audio-response)))
  "Returns string type for a service object of type 'behavior_talk_audio-response"
  "qt_robot_interface/behavior_talk_audioResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<behavior_talk_audio-response>)))
  "Returns md5sum for a message object of type '<behavior_talk_audio-response>"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'behavior_talk_audio-response)))
  "Returns md5sum for a message object of type 'behavior_talk_audio-response"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<behavior_talk_audio-response>)))
  "Returns full string definition for message of type '<behavior_talk_audio-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'behavior_talk_audio-response)))
  "Returns full string definition for message of type 'behavior_talk_audio-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <behavior_talk_audio-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <behavior_talk_audio-response>))
  "Converts a ROS message object to a list"
  (cl:list 'behavior_talk_audio-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'behavior_talk_audio)))
  'behavior_talk_audio-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'behavior_talk_audio)))
  'behavior_talk_audio-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'behavior_talk_audio)))
  "Returns string type for a service object of type '<behavior_talk_audio>"
  "qt_robot_interface/behavior_talk_audio")