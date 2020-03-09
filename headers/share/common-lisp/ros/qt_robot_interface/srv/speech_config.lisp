; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude speech_config-request.msg.html

(cl:defclass <speech_config-request> (roslisp-msg-protocol:ros-message)
  ((language
    :reader language
    :initarg :language
    :type cl:string
    :initform "")
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass speech_config-request (<speech_config-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_config-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_config-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<speech_config-request> is deprecated: use qt_robot_interface-srv:speech_config-request instead.")))

(cl:ensure-generic-function 'language-val :lambda-list '(m))
(cl:defmethod language-val ((m <speech_config-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:language-val is deprecated.  Use qt_robot_interface-srv:language instead.")
  (language m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <speech_config-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:pitch-val is deprecated.  Use qt_robot_interface-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <speech_config-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:speed-val is deprecated.  Use qt_robot_interface-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_config-request>) ostream)
  "Serializes a message object of type '<speech_config-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'language))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'language))
  (cl:let* ((signed (cl:slot-value msg 'pitch)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_config-request>) istream)
  "Deserializes a message object of type '<speech_config-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'language) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'language) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pitch) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_config-request>)))
  "Returns string type for a service object of type '<speech_config-request>"
  "qt_robot_interface/speech_configRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_config-request)))
  "Returns string type for a service object of type 'speech_config-request"
  "qt_robot_interface/speech_configRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_config-request>)))
  "Returns md5sum for a message object of type '<speech_config-request>"
  "bb0832ad049396f7ac17944e2242235c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_config-request)))
  "Returns md5sum for a message object of type 'speech_config-request"
  "bb0832ad049396f7ac17944e2242235c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_config-request>)))
  "Returns full string definition for message of type '<speech_config-request>"
  (cl:format cl:nil "string language~%int16  pitch~%int16  speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_config-request)))
  "Returns full string definition for message of type 'speech_config-request"
  (cl:format cl:nil "string language~%int16  pitch~%int16  speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_config-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'language))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_config-request>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_config-request
    (cl:cons ':language (language msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude speech_config-response.msg.html

(cl:defclass <speech_config-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass speech_config-response (<speech_config-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_config-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_config-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<speech_config-response> is deprecated: use qt_robot_interface-srv:speech_config-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <speech_config-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_config-response>) ostream)
  "Serializes a message object of type '<speech_config-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_config-response>) istream)
  "Deserializes a message object of type '<speech_config-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_config-response>)))
  "Returns string type for a service object of type '<speech_config-response>"
  "qt_robot_interface/speech_configResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_config-response)))
  "Returns string type for a service object of type 'speech_config-response"
  "qt_robot_interface/speech_configResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_config-response>)))
  "Returns md5sum for a message object of type '<speech_config-response>"
  "bb0832ad049396f7ac17944e2242235c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_config-response)))
  "Returns md5sum for a message object of type 'speech_config-response"
  "bb0832ad049396f7ac17944e2242235c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_config-response>)))
  "Returns full string definition for message of type '<speech_config-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_config-response)))
  "Returns full string definition for message of type 'speech_config-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_config-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_config-response>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_config-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'speech_config)))
  'speech_config-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'speech_config)))
  'speech_config-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_config)))
  "Returns string type for a service object of type '<speech_config>"
  "qt_robot_interface/speech_config")