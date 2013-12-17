; Auto-generated. Do not edit!


(cl:in-package zuros_sensors-msg)


;//! \htmlinclude Zwave.msg.html

(cl:defclass <Zwave> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (communication_id
    :reader communication_id
    :initarg :communication_id
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass Zwave (<Zwave>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Zwave>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Zwave)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name zuros_sensors-msg:<Zwave> is deprecated: use zuros_sensors-msg:Zwave instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Zwave>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_sensors-msg:name-val is deprecated.  Use zuros_sensors-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'communication_id-val :lambda-list '(m))
(cl:defmethod communication_id-val ((m <Zwave>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_sensors-msg:communication_id-val is deprecated.  Use zuros_sensors-msg:communication_id instead.")
  (communication_id m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Zwave>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_sensors-msg:value-val is deprecated.  Use zuros_sensors-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Zwave>) ostream)
  "Serializes a message object of type '<Zwave>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'communication_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'communication_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Zwave>) istream)
  "Deserializes a message object of type '<Zwave>"
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
      (cl:setf (cl:slot-value msg 'communication_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'communication_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Zwave>)))
  "Returns string type for a message object of type '<Zwave>"
  "zuros_sensors/Zwave")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Zwave)))
  "Returns string type for a message object of type 'Zwave"
  "zuros_sensors/Zwave")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Zwave>)))
  "Returns md5sum for a message object of type '<Zwave>"
  "795e66ef3e1f1ec8b6e126177b5aefce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Zwave)))
  "Returns md5sum for a message object of type 'Zwave"
  "795e66ef3e1f1ec8b6e126177b5aefce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Zwave>)))
  "Returns full string definition for message of type '<Zwave>"
  (cl:format cl:nil "string name~%string communication_id~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Zwave)))
  "Returns full string definition for message of type 'Zwave"
  (cl:format cl:nil "string name~%string communication_id~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Zwave>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'communication_id))
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Zwave>))
  "Converts a ROS message object to a list"
  (cl:list 'Zwave
    (cl:cons ':name (name msg))
    (cl:cons ':communication_id (communication_id msg))
    (cl:cons ':value (value msg))
))
