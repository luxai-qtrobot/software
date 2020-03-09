; Auto-generated. Do not edit!


(cl:in-package qt_motors_controller-srv)


;//! \htmlinclude set_control_mode-request.msg.html

(cl:defclass <set_control_mode-request> (roslisp-msg-protocol:ros-message)
  ((parts
    :reader parts
    :initarg :parts
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass set_control_mode-request (<set_control_mode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_control_mode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_control_mode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_motors_controller-srv:<set_control_mode-request> is deprecated: use qt_motors_controller-srv:set_control_mode-request instead.")))

(cl:ensure-generic-function 'parts-val :lambda-list '(m))
(cl:defmethod parts-val ((m <set_control_mode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:parts-val is deprecated.  Use qt_motors_controller-srv:parts instead.")
  (parts m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <set_control_mode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:mode-val is deprecated.  Use qt_motors_controller-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<set_control_mode-request>)))
    "Constants for message type '<set_control_mode-request>"
  '((:M_ON . 0)
    (:M_OFF . 1)
    (:M_BREAK . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'set_control_mode-request)))
    "Constants for message type 'set_control_mode-request"
  '((:M_ON . 0)
    (:M_OFF . 1)
    (:M_BREAK . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_control_mode-request>) ostream)
  "Serializes a message object of type '<set_control_mode-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_control_mode-request>) istream)
  "Deserializes a message object of type '<set_control_mode-request>"
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_control_mode-request>)))
  "Returns string type for a service object of type '<set_control_mode-request>"
  "qt_motors_controller/set_control_modeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_control_mode-request)))
  "Returns string type for a service object of type 'set_control_mode-request"
  "qt_motors_controller/set_control_modeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_control_mode-request>)))
  "Returns md5sum for a message object of type '<set_control_mode-request>"
  "29ec7411588be56f1e5fa3f64ddace43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_control_mode-request)))
  "Returns md5sum for a message object of type 'set_control_mode-request"
  "29ec7411588be56f1e5fa3f64ddace43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_control_mode-request>)))
  "Returns full string definition for message of type '<set_control_mode-request>"
  (cl:format cl:nil "~%uint8 M_ON=0~%uint8 M_OFF=1~%uint8 M_BREAK=2~%~%~%string[] parts~%uint8  mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_control_mode-request)))
  "Returns full string definition for message of type 'set_control_mode-request"
  (cl:format cl:nil "~%uint8 M_ON=0~%uint8 M_OFF=1~%uint8 M_BREAK=2~%~%~%string[] parts~%uint8  mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_control_mode-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'parts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_control_mode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_control_mode-request
    (cl:cons ':parts (parts msg))
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude set_control_mode-response.msg.html

(cl:defclass <set_control_mode-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_control_mode-response (<set_control_mode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_control_mode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_control_mode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_motors_controller-srv:<set_control_mode-response> is deprecated: use qt_motors_controller-srv:set_control_mode-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <set_control_mode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:status-val is deprecated.  Use qt_motors_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_control_mode-response>) ostream)
  "Serializes a message object of type '<set_control_mode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_control_mode-response>) istream)
  "Deserializes a message object of type '<set_control_mode-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_control_mode-response>)))
  "Returns string type for a service object of type '<set_control_mode-response>"
  "qt_motors_controller/set_control_modeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_control_mode-response)))
  "Returns string type for a service object of type 'set_control_mode-response"
  "qt_motors_controller/set_control_modeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_control_mode-response>)))
  "Returns md5sum for a message object of type '<set_control_mode-response>"
  "29ec7411588be56f1e5fa3f64ddace43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_control_mode-response)))
  "Returns md5sum for a message object of type 'set_control_mode-response"
  "29ec7411588be56f1e5fa3f64ddace43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_control_mode-response>)))
  "Returns full string definition for message of type '<set_control_mode-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_control_mode-response)))
  "Returns full string definition for message of type 'set_control_mode-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_control_mode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_control_mode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_control_mode-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_control_mode)))
  'set_control_mode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_control_mode)))
  'set_control_mode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_control_mode)))
  "Returns string type for a service object of type '<set_control_mode>"
  "qt_motors_controller/set_control_mode")