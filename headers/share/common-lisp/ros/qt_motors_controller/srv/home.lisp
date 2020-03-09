; Auto-generated. Do not edit!


(cl:in-package qt_motors_controller-srv)


;//! \htmlinclude home-request.msg.html

(cl:defclass <home-request> (roslisp-msg-protocol:ros-message)
  ((parts
    :reader parts
    :initarg :parts
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass home-request (<home-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <home-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'home-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_motors_controller-srv:<home-request> is deprecated: use qt_motors_controller-srv:home-request instead.")))

(cl:ensure-generic-function 'parts-val :lambda-list '(m))
(cl:defmethod parts-val ((m <home-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:parts-val is deprecated.  Use qt_motors_controller-srv:parts instead.")
  (parts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <home-request>) ostream)
  "Serializes a message object of type '<home-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <home-request>) istream)
  "Deserializes a message object of type '<home-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<home-request>)))
  "Returns string type for a service object of type '<home-request>"
  "qt_motors_controller/homeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'home-request)))
  "Returns string type for a service object of type 'home-request"
  "qt_motors_controller/homeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<home-request>)))
  "Returns md5sum for a message object of type '<home-request>"
  "018aa38faa01fecffa7a4eb6cad48ceb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'home-request)))
  "Returns md5sum for a message object of type 'home-request"
  "018aa38faa01fecffa7a4eb6cad48ceb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<home-request>)))
  "Returns full string definition for message of type '<home-request>"
  (cl:format cl:nil "~%string[] parts~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'home-request)))
  "Returns full string definition for message of type 'home-request"
  (cl:format cl:nil "~%string[] parts~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <home-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'parts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <home-request>))
  "Converts a ROS message object to a list"
  (cl:list 'home-request
    (cl:cons ':parts (parts msg))
))
;//! \htmlinclude home-response.msg.html

(cl:defclass <home-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass home-response (<home-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <home-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'home-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_motors_controller-srv:<home-response> is deprecated: use qt_motors_controller-srv:home-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <home-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_motors_controller-srv:status-val is deprecated.  Use qt_motors_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <home-response>) ostream)
  "Serializes a message object of type '<home-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <home-response>) istream)
  "Deserializes a message object of type '<home-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<home-response>)))
  "Returns string type for a service object of type '<home-response>"
  "qt_motors_controller/homeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'home-response)))
  "Returns string type for a service object of type 'home-response"
  "qt_motors_controller/homeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<home-response>)))
  "Returns md5sum for a message object of type '<home-response>"
  "018aa38faa01fecffa7a4eb6cad48ceb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'home-response)))
  "Returns md5sum for a message object of type 'home-response"
  "018aa38faa01fecffa7a4eb6cad48ceb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<home-response>)))
  "Returns full string definition for message of type '<home-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'home-response)))
  "Returns full string definition for message of type 'home-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <home-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <home-response>))
  "Converts a ROS message object to a list"
  (cl:list 'home-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'home)))
  'home-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'home)))
  'home-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'home)))
  "Returns string type for a service object of type '<home>"
  "qt_motors_controller/home")