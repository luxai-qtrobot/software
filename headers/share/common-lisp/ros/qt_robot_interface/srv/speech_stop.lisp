; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude speech_stop-request.msg.html

(cl:defclass <speech_stop-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass speech_stop-request (<speech_stop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_stop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_stop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<speech_stop-request> is deprecated: use qt_robot_interface-srv:speech_stop-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_stop-request>) ostream)
  "Serializes a message object of type '<speech_stop-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_stop-request>) istream)
  "Deserializes a message object of type '<speech_stop-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_stop-request>)))
  "Returns string type for a service object of type '<speech_stop-request>"
  "qt_robot_interface/speech_stopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_stop-request)))
  "Returns string type for a service object of type 'speech_stop-request"
  "qt_robot_interface/speech_stopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_stop-request>)))
  "Returns md5sum for a message object of type '<speech_stop-request>"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_stop-request)))
  "Returns md5sum for a message object of type 'speech_stop-request"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_stop-request>)))
  "Returns full string definition for message of type '<speech_stop-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_stop-request)))
  "Returns full string definition for message of type 'speech_stop-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_stop-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_stop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_stop-request
))
;//! \htmlinclude speech_stop-response.msg.html

(cl:defclass <speech_stop-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass speech_stop-response (<speech_stop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_stop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_stop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<speech_stop-response> is deprecated: use qt_robot_interface-srv:speech_stop-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <speech_stop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_stop-response>) ostream)
  "Serializes a message object of type '<speech_stop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_stop-response>) istream)
  "Deserializes a message object of type '<speech_stop-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_stop-response>)))
  "Returns string type for a service object of type '<speech_stop-response>"
  "qt_robot_interface/speech_stopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_stop-response)))
  "Returns string type for a service object of type 'speech_stop-response"
  "qt_robot_interface/speech_stopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_stop-response>)))
  "Returns md5sum for a message object of type '<speech_stop-response>"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_stop-response)))
  "Returns md5sum for a message object of type 'speech_stop-response"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_stop-response>)))
  "Returns full string definition for message of type '<speech_stop-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_stop-response)))
  "Returns full string definition for message of type 'speech_stop-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_stop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_stop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_stop-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'speech_stop)))
  'speech_stop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'speech_stop)))
  'speech_stop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_stop)))
  "Returns string type for a service object of type '<speech_stop>"
  "qt_robot_interface/speech_stop")