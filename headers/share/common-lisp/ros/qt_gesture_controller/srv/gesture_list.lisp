; Auto-generated. Do not edit!


(cl:in-package qt_gesture_controller-srv)


;//! \htmlinclude gesture_list-request.msg.html

(cl:defclass <gesture_list-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass gesture_list-request (<gesture_list-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_list-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_list-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_list-request> is deprecated: use qt_gesture_controller-srv:gesture_list-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_list-request>) ostream)
  "Serializes a message object of type '<gesture_list-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_list-request>) istream)
  "Deserializes a message object of type '<gesture_list-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_list-request>)))
  "Returns string type for a service object of type '<gesture_list-request>"
  "qt_gesture_controller/gesture_listRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_list-request)))
  "Returns string type for a service object of type 'gesture_list-request"
  "qt_gesture_controller/gesture_listRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_list-request>)))
  "Returns md5sum for a message object of type '<gesture_list-request>"
  "fff31f89d10c3103fdbdb3a5212feb94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_list-request)))
  "Returns md5sum for a message object of type 'gesture_list-request"
  "fff31f89d10c3103fdbdb3a5212feb94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_list-request>)))
  "Returns full string definition for message of type '<gesture_list-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_list-request)))
  "Returns full string definition for message of type 'gesture_list-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_list-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_list-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_list-request
))
;//! \htmlinclude gesture_list-response.msg.html

(cl:defclass <gesture_list-response> (roslisp-msg-protocol:ros-message)
  ((gestures
    :reader gestures
    :initarg :gestures
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gesture_list-response (<gesture_list-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_list-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_list-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_list-response> is deprecated: use qt_gesture_controller-srv:gesture_list-response instead.")))

(cl:ensure-generic-function 'gestures-val :lambda-list '(m))
(cl:defmethod gestures-val ((m <gesture_list-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:gestures-val is deprecated.  Use qt_gesture_controller-srv:gestures instead.")
  (gestures m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <gesture_list-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:status-val is deprecated.  Use qt_gesture_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_list-response>) ostream)
  "Serializes a message object of type '<gesture_list-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gestures))))
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
   (cl:slot-value msg 'gestures))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_list-response>) istream)
  "Deserializes a message object of type '<gesture_list-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gestures) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gestures)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_list-response>)))
  "Returns string type for a service object of type '<gesture_list-response>"
  "qt_gesture_controller/gesture_listResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_list-response)))
  "Returns string type for a service object of type 'gesture_list-response"
  "qt_gesture_controller/gesture_listResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_list-response>)))
  "Returns md5sum for a message object of type '<gesture_list-response>"
  "fff31f89d10c3103fdbdb3a5212feb94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_list-response)))
  "Returns md5sum for a message object of type 'gesture_list-response"
  "fff31f89d10c3103fdbdb3a5212feb94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_list-response>)))
  "Returns full string definition for message of type '<gesture_list-response>"
  (cl:format cl:nil "string[] gestures~%bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_list-response)))
  "Returns full string definition for message of type 'gesture_list-response"
  (cl:format cl:nil "string[] gestures~%bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_list-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gestures) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_list-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_list-response
    (cl:cons ':gestures (gestures msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gesture_list)))
  'gesture_list-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gesture_list)))
  'gesture_list-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_list)))
  "Returns string type for a service object of type '<gesture_list>"
  "qt_gesture_controller/gesture_list")