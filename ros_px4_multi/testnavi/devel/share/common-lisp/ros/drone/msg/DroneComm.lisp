; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude DroneComm.msg.html

(cl:defclass <DroneComm> (roslisp-msg-protocol:ros-message)
  ((d1
    :reader d1
    :initarg :d1
    :type cl:integer
    :initform 0)
   (d2
    :reader d2
    :initarg :d2
    :type cl:integer
    :initform 0))
)

(cl:defclass DroneComm (<DroneComm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneComm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneComm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<DroneComm> is deprecated: use drone-msg:DroneComm instead.")))

(cl:ensure-generic-function 'd1-val :lambda-list '(m))
(cl:defmethod d1-val ((m <DroneComm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:d1-val is deprecated.  Use drone-msg:d1 instead.")
  (d1 m))

(cl:ensure-generic-function 'd2-val :lambda-list '(m))
(cl:defmethod d2-val ((m <DroneComm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:d2-val is deprecated.  Use drone-msg:d2 instead.")
  (d2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneComm>) ostream)
  "Serializes a message object of type '<DroneComm>"
  (cl:let* ((signed (cl:slot-value msg 'd1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'd2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneComm>) istream)
  "Deserializes a message object of type '<DroneComm>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'd1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'd2) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneComm>)))
  "Returns string type for a message object of type '<DroneComm>"
  "drone/DroneComm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneComm)))
  "Returns string type for a message object of type 'DroneComm"
  "drone/DroneComm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneComm>)))
  "Returns md5sum for a message object of type '<DroneComm>"
  "6709d43b931cab88bf24bfce91f3f579")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneComm)))
  "Returns md5sum for a message object of type 'DroneComm"
  "6709d43b931cab88bf24bfce91f3f579")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneComm>)))
  "Returns full string definition for message of type '<DroneComm>"
  (cl:format cl:nil "int64 d1~%int64 d2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneComm)))
  "Returns full string definition for message of type 'DroneComm"
  (cl:format cl:nil "int64 d1~%int64 d2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneComm>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneComm>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneComm
    (cl:cons ':d1 (d1 msg))
    (cl:cons ':d2 (d2 msg))
))
