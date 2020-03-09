; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude audio_play-request.msg.html

(cl:defclass <audio_play-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass audio_play-request (<audio_play-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <audio_play-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'audio_play-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<audio_play-request> is deprecated: use qt_robot_interface-srv:audio_play-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <audio_play-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:filename-val is deprecated.  Use qt_robot_interface-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'filepath-val :lambda-list '(m))
(cl:defmethod filepath-val ((m <audio_play-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:filepath-val is deprecated.  Use qt_robot_interface-srv:filepath instead.")
  (filepath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <audio_play-request>) ostream)
  "Serializes a message object of type '<audio_play-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <audio_play-request>) istream)
  "Deserializes a message object of type '<audio_play-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<audio_play-request>)))
  "Returns string type for a service object of type '<audio_play-request>"
  "qt_robot_interface/audio_playRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'audio_play-request)))
  "Returns string type for a service object of type 'audio_play-request"
  "qt_robot_interface/audio_playRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<audio_play-request>)))
  "Returns md5sum for a message object of type '<audio_play-request>"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'audio_play-request)))
  "Returns md5sum for a message object of type 'audio_play-request"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<audio_play-request>)))
  "Returns full string definition for message of type '<audio_play-request>"
  (cl:format cl:nil "string filename~%string filepath~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'audio_play-request)))
  "Returns full string definition for message of type 'audio_play-request"
  (cl:format cl:nil "string filename~%string filepath~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <audio_play-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'filepath))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <audio_play-request>))
  "Converts a ROS message object to a list"
  (cl:list 'audio_play-request
    (cl:cons ':filename (filename msg))
    (cl:cons ':filepath (filepath msg))
))
;//! \htmlinclude audio_play-response.msg.html

(cl:defclass <audio_play-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass audio_play-response (<audio_play-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <audio_play-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'audio_play-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<audio_play-response> is deprecated: use qt_robot_interface-srv:audio_play-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <audio_play-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <audio_play-response>) ostream)
  "Serializes a message object of type '<audio_play-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <audio_play-response>) istream)
  "Deserializes a message object of type '<audio_play-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<audio_play-response>)))
  "Returns string type for a service object of type '<audio_play-response>"
  "qt_robot_interface/audio_playResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'audio_play-response)))
  "Returns string type for a service object of type 'audio_play-response"
  "qt_robot_interface/audio_playResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<audio_play-response>)))
  "Returns md5sum for a message object of type '<audio_play-response>"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'audio_play-response)))
  "Returns md5sum for a message object of type 'audio_play-response"
  "52268a262dc8cf03d5103995c9714ffa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<audio_play-response>)))
  "Returns full string definition for message of type '<audio_play-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'audio_play-response)))
  "Returns full string definition for message of type 'audio_play-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <audio_play-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <audio_play-response>))
  "Converts a ROS message object to a list"
  (cl:list 'audio_play-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'audio_play)))
  'audio_play-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'audio_play)))
  'audio_play-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'audio_play)))
  "Returns string type for a service object of type '<audio_play>"
  "qt_robot_interface/audio_play")