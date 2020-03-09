; Auto-generated. Do not edit!


(cl:in-package qt_motors_controller-srv)


;//! \htmlinclude set_velocity-request.msg.html

(cl:defclass <set_velocity-request> (roslisp-msg-protocol:ros-message)
  ((parts
    :reader parts
    :initarg :parts
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:fixnum
    :initform 0))
)

(cl:defclass set_velocity-request (<set_velocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_velocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_velocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_motors_controller-srv:<set_velocity-request> is deprecated: use qt_motors_controller-srv:set_velocity-request instead.")))

(cl:ensure-generic-function 'parts-val :lambda-list '(m))
(cl:defmethod parts-val ((m <set_velocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:parts-val is deprecated.  Use qt_motors_controller-srv:parts instead.")
  (parts m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <set_velocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:velocity-val is deprecated.  Use qt_motors_controller-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_velocity-request>) ostream)
  "Serializes a message object of type '<set_velocity-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'parts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'parts))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'velocity)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_velocity-request>) istream)
  "Deserializes a message object of type '<set_velocity-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'parts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'parts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_velocity-request>)))
  "Returns string type for a service object of type '<set_velocity-request>"
  "qt_motors_controller/set_velocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_velocity-request)))
  "Returns string type for a service object of type 'set_velocity-request"
  "qt_motors_controller/set_velocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_velocity-request>)))
  "Returns md5sum for a message object of type '<set_velocity-request>"
  "68003f66a5a441a7e064a7fc5cd19661")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_velocity-request)))
  "Returns md5sum for a message object of type 'set_velocity-request"
  "68003f66a5a441a7e064a7fc5cd19661")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_velocity-request>)))
  "Returns full string definition for message of type '<set_velocity-request>"
  (cl:format cl:nil "~%string[] parts~%uint8  velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_velocity-request)))
  "Returns full string definition for message of type 'set_velocity-request"
  (cl:format cl:nil "~%string[] parts~%uint8  velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_velocity-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'parts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_velocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_velocity-request
    (cl:cons ':parts (parts msg))
    (cl:cons ':velocity (velocity msg))
))
;//! \htmlinclude set_velocity-response.msg.html

(cl:defclass <set_velocity-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_velocity-response (<set_velocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_velocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_velocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_motors_controller-srv:<set_velocity-response> is deprecated: use qt_motors_controller-srv:set_velocity-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <set_velocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:status-val is deprecated.  Use qt_motors_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_velocity-response>) ostream)
  "Serializes a message object of type '<set_velocity-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_velocity-response>) istream)
  "Deserializes a message object of type '<set_velocity-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_velocity-response>)))
  "Returns string type for a service object of type '<set_velocity-response>"
  "qt_motors_controller/set_velocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_velocity-response)))
  "Returns string type for a service object of type 'set_velocity-response"
  "qt_motors_controller/set_velocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_velocity-response>)))
  "Returns md5sum for a message object of type '<set_velocity-response>"
  "68003f66a5a441a7e064a7fc5cd19661")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_velocity-response)))
  "Returns md5sum for a message object of type 'set_velocity-response"
  "68003f66a5a441a7e064a7fc5cd19661")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_velocity-response>)))
  "Returns full string definition for message of type '<set_velocity-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_velocity-response)))
  "Returns full string definition for message of type 'set_velocity-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_velocity-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_velocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_velocity-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_velocity)))
  'set_velocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_velocity)))
  'set_velocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_velocity)))
  "Returns string type for a service object of type '<set_velocity>"
  "qt_motors_controller/set_velocity")