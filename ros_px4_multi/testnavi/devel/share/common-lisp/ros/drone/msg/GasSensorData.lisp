; Auto-generated. Do not edit!


(cl:in-package drone-msg)


;//! \htmlinclude GasSensorData.msg.html

(cl:defclass <GasSensorData> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass GasSensorData (<GasSensorData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GasSensorData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GasSensorData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone-msg:<GasSensorData> is deprecated: use drone-msg:GasSensorData instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GasSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone-msg:value-val is deprecated.  Use drone-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GasSensorData>) ostream)
  "Serializes a message object of type '<GasSensorData>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GasSensorData>) istream)
  "Deserializes a message object of type '<GasSensorData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GasSensorData>)))
  "Returns string type for a message object of type '<GasSensorData>"
  "drone/GasSensorData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GasSensorData)))
  "Returns string type for a message object of type 'GasSensorData"
  "drone/GasSensorData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GasSensorData>)))
  "Returns md5sum for a message object of type '<GasSensorData>"
  "1b1594d2b74931ef8fe7be8e2d594455")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GasSensorData)))
  "Returns md5sum for a message object of type 'GasSensorData"
  "1b1594d2b74931ef8fe7be8e2d594455")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GasSensorData>)))
  "Returns full string definition for message of type '<GasSensorData>"
  (cl:format cl:nil "float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GasSensorData)))
  "Returns full string definition for message of type 'GasSensorData"
  (cl:format cl:nil "float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GasSensorData>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GasSensorData>))
  "Converts a ROS message object to a list"
  (cl:list 'GasSensorData
    (cl:cons ':value (value msg))
))
