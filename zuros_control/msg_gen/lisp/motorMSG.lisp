; Auto-generated. Do not edit!


(cl:in-package zuros_control-msg)


;//! \htmlinclude motorMSG.msg.html

(cl:defclass <motorMSG> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (accel
    :reader accel
    :initarg :accel
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass motorMSG (<motorMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name zuros_control-msg:<motorMSG> is deprecated: use zuros_control-msg:motorMSG instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_control-msg:id-val is deprecated.  Use zuros_control-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_control-msg:mode-val is deprecated.  Use zuros_control-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_control-msg:accel-val is deprecated.  Use zuros_control-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_control-msg:speed-val is deprecated.  Use zuros_control-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorMSG>) ostream)
  "Serializes a message object of type '<motorMSG>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (cl:let* ((signed (cl:slot-value msg 'accel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorMSG>) istream)
  "Deserializes a message object of type '<motorMSG>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'accel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorMSG>)))
  "Returns string type for a message object of type '<motorMSG>"
  "zuros_control/motorMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorMSG)))
  "Returns string type for a message object of type 'motorMSG"
  "zuros_control/motorMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorMSG>)))
  "Returns md5sum for a message object of type '<motorMSG>"
  "2b842696bf7d178aec69e2f08b9dd853")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorMSG)))
  "Returns md5sum for a message object of type 'motorMSG"
  "2b842696bf7d178aec69e2f08b9dd853")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorMSG>)))
  "Returns full string definition for message of type '<motorMSG>"
  (cl:format cl:nil "int32 id~%string mode~%int32 accel~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorMSG)))
  "Returns full string definition for message of type 'motorMSG"
  (cl:format cl:nil "int32 id~%string mode~%int32 accel~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorMSG>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'mode))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'motorMSG
    (cl:cons ':id (id msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':accel (accel msg))
    (cl:cons ':speed (speed msg))
))
