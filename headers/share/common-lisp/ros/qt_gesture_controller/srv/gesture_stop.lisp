; Auto-generated. Do not edit!


(cl:in-package qt_gesture_controller-srv)


;//! \htmlinclude gesture_stop-request.msg.html

(cl:defclass <gesture_stop-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass gesture_stop-request (<gesture_stop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_stop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_stop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_stop-request> is deprecated: use qt_gesture_controller-srv:gesture_stop-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_stop-request>) ostream)
  "Serializes a message object of type '<gesture_stop-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_stop-request>) istream)
  "Deserializes a message object of type '<gesture_stop-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_stop-request>)))
  "Returns string type for a service object of type '<gesture_stop-request>"
  "qt_gesture_controller/gesture_stopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_stop-request)))
  "Returns string type for a service object of type 'gesture_stop-request"
  "qt_gesture_controller/gesture_stopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_stop-request>)))
  "Returns md5sum for a message object of type '<gesture_stop-request>"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_stop-request)))
  "Returns md5sum for a message object of type 'gesture_stop-request"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_stop-request>)))
  "Returns full string definition for message of type '<gesture_stop-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_stop-request)))
  "Returns full string definition for message of type 'gesture_stop-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_stop-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_stop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_stop-request
))
;//! \htmlinclude gesture_stop-response.msg.html

(cl:defclass <gesture_stop-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gesture_stop-response (<gesture_stop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_stop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_stop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_stop-response> is deprecated: use qt_gesture_controller-srv:gesture_stop-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <gesture_stop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:status-val is deprecated.  Use qt_gesture_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_stop-response>) ostream)
  "Serializes a message object of type '<gesture_stop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_stop-response>) istream)
  "Deserializes a message object of type '<gesture_stop-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_stop-response>)))
  "Returns string type for a service object of type '<gesture_stop-response>"
  "qt_gesture_controller/gesture_stopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_stop-response)))
  "Returns string type for a service object of type 'gesture_stop-response"
  "qt_gesture_controller/gesture_stopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_stop-response>)))
  "Returns md5sum for a message object of type '<gesture_stop-response>"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_stop-response)))
  "Returns md5sum for a message object of type 'gesture_stop-response"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_stop-response>)))
  "Returns full string definition for message of type '<gesture_stop-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_stop-response)))
  "Returns full string definition for message of type 'gesture_stop-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_stop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_stop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_stop-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gesture_stop)))
  'gesture_stop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gesture_stop)))
  'gesture_stop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_stop)))
  "Returns string type for a service object of type '<gesture_stop>"
  "qt_gesture_controller/gesture_stop")