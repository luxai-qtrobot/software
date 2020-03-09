; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude emotion_show-request.msg.html

(cl:defclass <emotion_show-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass emotion_show-request (<emotion_show-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emotion_show-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emotion_show-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<emotion_show-request> is deprecated: use qt_robot_interface-srv:emotion_show-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <emotion_show-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:name-val is deprecated.  Use qt_robot_interface-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emotion_show-request>) ostream)
  "Serializes a message object of type '<emotion_show-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emotion_show-request>) istream)
  "Deserializes a message object of type '<emotion_show-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emotion_show-request>)))
  "Returns string type for a service object of type '<emotion_show-request>"
  "qt_robot_interface/emotion_showRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emotion_show-request)))
  "Returns string type for a service object of type 'emotion_show-request"
  "qt_robot_interface/emotion_showRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emotion_show-request>)))
  "Returns md5sum for a message object of type '<emotion_show-request>"
  "186befe2a32d448f6a8e15271a5e2624")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emotion_show-request)))
  "Returns md5sum for a message object of type 'emotion_show-request"
  "186befe2a32d448f6a8e15271a5e2624")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emotion_show-request>)))
  "Returns full string definition for message of type '<emotion_show-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emotion_show-request)))
  "Returns full string definition for message of type 'emotion_show-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emotion_show-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emotion_show-request>))
  "Converts a ROS message object to a list"
  (cl:list 'emotion_show-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude emotion_show-response.msg.html

(cl:defclass <emotion_show-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass emotion_show-response (<emotion_show-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emotion_show-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emotion_show-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<emotion_show-response> is deprecated: use qt_robot_interface-srv:emotion_show-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <emotion_show-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emotion_show-response>) ostream)
  "Serializes a message object of type '<emotion_show-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emotion_show-response>) istream)
  "Deserializes a message object of type '<emotion_show-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emotion_show-response>)))
  "Returns string type for a service object of type '<emotion_show-response>"
  "qt_robot_interface/emotion_showResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emotion_show-response)))
  "Returns string type for a service object of type 'emotion_show-response"
  "qt_robot_interface/emotion_showResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emotion_show-response>)))
  "Returns md5sum for a message object of type '<emotion_show-response>"
  "186befe2a32d448f6a8e15271a5e2624")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emotion_show-response)))
  "Returns md5sum for a message object of type 'emotion_show-response"
  "186befe2a32d448f6a8e15271a5e2624")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emotion_show-response>)))
  "Returns full string definition for message of type '<emotion_show-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emotion_show-response)))
  "Returns full string definition for message of type 'emotion_show-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emotion_show-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emotion_show-response>))
  "Converts a ROS message object to a list"
  (cl:list 'emotion_show-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'emotion_show)))
  'emotion_show-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'emotion_show)))
  'emotion_show-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emotion_show)))
  "Returns string type for a service object of type '<emotion_show>"
  "qt_robot_interface/emotion_show")