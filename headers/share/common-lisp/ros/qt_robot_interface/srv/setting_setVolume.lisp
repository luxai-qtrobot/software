; Auto-generated. Do not edit!


(cl:in-package qt_robot_interface-srv)


;//! \htmlinclude setting_setVolume-request.msg.html

(cl:defclass <setting_setVolume-request> (roslisp-msg-protocol:ros-message)
  ((volume
    :reader volume
    :initarg :volume
    :type cl:fixnum
    :initform 0))
)

(cl:defclass setting_setVolume-request (<setting_setVolume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setting_setVolume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setting_setVolume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<setting_setVolume-request> is deprecated: use qt_robot_interface-srv:setting_setVolume-request instead.")))

(cl:ensure-generic-function 'volume-val :lambda-list '(m))
(cl:defmethod volume-val ((m <setting_setVolume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:volume-val is deprecated.  Use qt_robot_interface-srv:volume instead.")
  (volume m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setting_setVolume-request>) ostream)
  "Serializes a message object of type '<setting_setVolume-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'volume)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setting_setVolume-request>) istream)
  "Deserializes a message object of type '<setting_setVolume-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'volume)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setting_setVolume-request>)))
  "Returns string type for a service object of type '<setting_setVolume-request>"
  "qt_robot_interface/setting_setVolumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setting_setVolume-request)))
  "Returns string type for a service object of type 'setting_setVolume-request"
  "qt_robot_interface/setting_setVolumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setting_setVolume-request>)))
  "Returns md5sum for a message object of type '<setting_setVolume-request>"
  "14f0bfd003d9dc3318a211b307c7e7ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setting_setVolume-request)))
  "Returns md5sum for a message object of type 'setting_setVolume-request"
  "14f0bfd003d9dc3318a211b307c7e7ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setting_setVolume-request>)))
  "Returns full string definition for message of type '<setting_setVolume-request>"
  (cl:format cl:nil "uint8 volume~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setting_setVolume-request)))
  "Returns full string definition for message of type 'setting_setVolume-request"
  (cl:format cl:nil "uint8 volume~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setting_setVolume-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setting_setVolume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setting_setVolume-request
    (cl:cons ':volume (volume msg))
))
;//! \htmlinclude setting_setVolume-response.msg.html

(cl:defclass <setting_setVolume-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setting_setVolume-response (<setting_setVolume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setting_setVolume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setting_setVolume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_robot_interface-srv:<setting_setVolume-response> is deprecated: use qt_robot_interface-srv:setting_setVolume-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <setting_setVolume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_robot_interface-srv:status-val is deprecated.  Use qt_robot_interface-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setting_setVolume-response>) ostream)
  "Serializes a message object of type '<setting_setVolume-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setting_setVolume-response>) istream)
  "Deserializes a message object of type '<setting_setVolume-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setting_setVolume-response>)))
  "Returns string type for a service object of type '<setting_setVolume-response>"
  "qt_robot_interface/setting_setVolumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setting_setVolume-response)))
  "Returns string type for a service object of type 'setting_setVolume-response"
  "qt_robot_interface/setting_setVolumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setting_setVolume-response>)))
  "Returns md5sum for a message object of type '<setting_setVolume-response>"
  "14f0bfd003d9dc3318a211b307c7e7ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setting_setVolume-response)))
  "Returns md5sum for a message object of type 'setting_setVolume-response"
  "14f0bfd003d9dc3318a211b307c7e7ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setting_setVolume-response>)))
  "Returns full string definition for message of type '<setting_setVolume-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setting_setVolume-response)))
  "Returns full string definition for message of type 'setting_setVolume-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setting_setVolume-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setting_setVolume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setting_setVolume-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setting_setVolume)))
  'setting_setVolume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setting_setVolume)))
  'setting_setVolume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setting_setVolume)))
  "Returns string type for a service object of type '<setting_setVolume>"
  "qt_robot_interface/setting_setVolume")