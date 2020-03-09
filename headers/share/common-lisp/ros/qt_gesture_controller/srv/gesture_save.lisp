; Auto-generated. Do not edit!


(cl:in-package qt_gesture_controller-srv)


;//! \htmlinclude gesture_save-request.msg.html

(cl:defclass <gesture_save-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass gesture_save-request (<gesture_save-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_save-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_save-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_save-request> is deprecated: use qt_gesture_controller-srv:gesture_save-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <gesture_save-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:name-val is deprecated.  Use qt_gesture_controller-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <gesture_save-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:path-val is deprecated.  Use qt_gesture_controller-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_save-request>) ostream)
  "Serializes a message object of type '<gesture_save-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_save-request>) istream)
  "Deserializes a message object of type '<gesture_save-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_save-request>)))
  "Returns string type for a service object of type '<gesture_save-request>"
  "qt_gesture_controller/gesture_saveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_save-request)))
  "Returns string type for a service object of type 'gesture_save-request"
  "qt_gesture_controller/gesture_saveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_save-request>)))
  "Returns md5sum for a message object of type '<gesture_save-request>"
  "088f4facf8938c7925f9bdd7cacce95b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_save-request)))
  "Returns md5sum for a message object of type 'gesture_save-request"
  "088f4facf8938c7925f9bdd7cacce95b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_save-request>)))
  "Returns full string definition for message of type '<gesture_save-request>"
  (cl:format cl:nil "~%string name~%~%~%string path~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_save-request)))
  "Returns full string definition for message of type 'gesture_save-request"
  (cl:format cl:nil "~%string name~%~%~%string path~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_save-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_save-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_save-request
    (cl:cons ':name (name msg))
    (cl:cons ':path (path msg))
))
;//! \htmlinclude gesture_save-response.msg.html

(cl:defclass <gesture_save-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gesture_save-response (<gesture_save-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gesture_save-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gesture_save-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_gesture_controller-srv:<gesture_save-response> is deprecated: use qt_gesture_controller-srv:gesture_save-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <gesture_save-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_gesture_controller-srv:status-val is deprecated.  Use qt_gesture_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gesture_save-response>) ostream)
  "Serializes a message object of type '<gesture_save-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gesture_save-response>) istream)
  "Deserializes a message object of type '<gesture_save-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gesture_save-response>)))
  "Returns string type for a service object of type '<gesture_save-response>"
  "qt_gesture_controller/gesture_saveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_save-response)))
  "Returns string type for a service object of type 'gesture_save-response"
  "qt_gesture_controller/gesture_saveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gesture_save-response>)))
  "Returns md5sum for a message object of type '<gesture_save-response>"
  "088f4facf8938c7925f9bdd7cacce95b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gesture_save-response)))
  "Returns md5sum for a message object of type 'gesture_save-response"
  "088f4facf8938c7925f9bdd7cacce95b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gesture_save-response>)))
  "Returns full string definition for message of type '<gesture_save-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gesture_save-response)))
  "Returns full string definition for message of type 'gesture_save-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gesture_save-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gesture_save-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gesture_save-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gesture_save)))
  'gesture_save-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gesture_save)))
  'gesture_save-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gesture_save)))
  "Returns string type for a service object of type '<gesture_save>"
  "qt_gesture_controller/gesture_save")