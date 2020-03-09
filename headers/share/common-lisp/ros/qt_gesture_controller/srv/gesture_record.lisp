; Auto-generated. Do not edit!


(cl:in-package qt_gesture_controller-srv)


;//! \htmlinclude gesture_record-request.msg.html

(cl:defclass <gesture_record-request> (roslisp-msg-protocol:ros-message)
  ((parts
    :reader parts
    :initarg :parts
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (idleParts
    :reader idleParts
    :initarg :idleParts
    :type cl:boolean
    :initform cl:nil)
   (wait
    :reader wait
    :initarg :wait
    :type cl:fixnum
    :initform 0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:fixnum
    :initform 0))
)

(cl:defclass gesture_record-request (<gesture_record-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_record-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_record-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_record-request> is deprecated: use qt_gesture_controller-srv:gesture_record-request instead.")))

(cl:ensure-generic-function 'parts-val :lambda-list '(m))
(cl:defmethod parts-val ((m <gesture_record-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:parts-val is deprecated.  Use qt_gesture_controller-srv:parts instead.")
  (parts m))

(cl:ensure-generic-function 'idleParts-val :lambda-list '(m))
(cl:defmethod idleParts-val ((m <gesture_record-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:idleParts-val is deprecated.  Use qt_gesture_controller-srv:idleParts instead.")
  (idleParts m))

(cl:ensure-generic-function 'wait-val :lambda-list '(m))
(cl:defmethod wait-val ((m <gesture_record-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:wait-val is deprecated.  Use qt_gesture_controller-srv:wait instead.")
  (wait m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <gesture_record-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:timeout-val is deprecated.  Use qt_gesture_controller-srv:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_record-request>) ostream)
  "Serializes a message object of type '<gesture_record-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'idleParts) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wait)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_record-request>) istream)
  "Deserializes a message object of type '<gesture_record-request>"
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
    (cl:setf (cl:slot-value msg 'idleParts) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wait)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timeout)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_record-request>)))
  "Returns string type for a service object of type '<gesture_record-request>"
  "qt_gesture_controller/gesture_recordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_record-request)))
  "Returns string type for a service object of type 'gesture_record-request"
  "qt_gesture_controller/gesture_recordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_record-request>)))
  "Returns md5sum for a message object of type '<gesture_record-request>"
  "f920c6a17216e8fe1e02e50dada8c9d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_record-request)))
  "Returns md5sum for a message object of type 'gesture_record-request"
  "f920c6a17216e8fe1e02e50dada8c9d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_record-request>)))
  "Returns full string definition for message of type '<gesture_record-request>"
  (cl:format cl:nil "~%string[] parts~%~%~%bool idleParts~%~%~%uint8 wait~%~%~%uint8 timeout~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_record-request)))
  "Returns full string definition for message of type 'gesture_record-request"
  (cl:format cl:nil "~%string[] parts~%~%~%bool idleParts~%~%~%uint8 wait~%~%~%uint8 timeout~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_record-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'parts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_record-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_record-request
    (cl:cons ':parts (parts msg))
    (cl:cons ':idleParts (idleParts msg))
    (cl:cons ':wait (wait msg))
    (cl:cons ':timeout (timeout msg))
))
;//! \htmlinclude gesture_record-response.msg.html

(cl:defclass <gesture_record-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gesture_record-response (<gesture_record-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_record-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_record-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_record-response> is deprecated: use qt_gesture_controller-srv:gesture_record-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <gesture_record-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:status-val is deprecated.  Use qt_gesture_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_record-response>) ostream)
  "Serializes a message object of type '<gesture_record-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_record-response>) istream)
  "Deserializes a message object of type '<gesture_record-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_record-response>)))
  "Returns string type for a service object of type '<gesture_record-response>"
  "qt_gesture_controller/gesture_recordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_record-response)))
  "Returns string type for a service object of type 'gesture_record-response"
  "qt_gesture_controller/gesture_recordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_record-response>)))
  "Returns md5sum for a message object of type '<gesture_record-response>"
  "f920c6a17216e8fe1e02e50dada8c9d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_record-response)))
  "Returns md5sum for a message object of type 'gesture_record-response"
  "f920c6a17216e8fe1e02e50dada8c9d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_record-response>)))
  "Returns full string definition for message of type '<gesture_record-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_record-response)))
  "Returns full string definition for message of type 'gesture_record-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_record-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_record-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_record-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gesture_record)))
  'gesture_record-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gesture_record)))
  'gesture_record-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_record)))
  "Returns string type for a service object of type '<gesture_record>"
  "qt_gesture_controller/gesture_record")