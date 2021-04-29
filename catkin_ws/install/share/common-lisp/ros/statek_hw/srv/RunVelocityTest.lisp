; Auto-generated. Do not edit!


(cl:in-package statek_hw-srv)


;//! \htmlinclude RunVelocityTest-request.msg.html

(cl:defclass <RunVelocityTest-request> (roslisp-msg-protocol:ros-message)
  ((time_ms
    :reader time_ms
    :initarg :time_ms
    :type cl:integer
    :initform 0))
)

(cl:defclass RunVelocityTest-request (<RunVelocityTest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunVelocityTest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunVelocityTest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<RunVelocityTest-request> is deprecated: use statek_hw-srv:RunVelocityTest-request instead.")))

(cl:ensure-generic-function 'time_ms-val :lambda-list '(m))
(cl:defmethod time_ms-val ((m <RunVelocityTest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:time_ms-val is deprecated.  Use statek_hw-srv:time_ms instead.")
  (time_ms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunVelocityTest-request>) ostream)
  "Serializes a message object of type '<RunVelocityTest-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_ms)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunVelocityTest-request>) istream)
  "Deserializes a message object of type '<RunVelocityTest-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_ms)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunVelocityTest-request>)))
  "Returns string type for a service object of type '<RunVelocityTest-request>"
  "statek_hw/RunVelocityTestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunVelocityTest-request)))
  "Returns string type for a service object of type 'RunVelocityTest-request"
  "statek_hw/RunVelocityTestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunVelocityTest-request>)))
  "Returns md5sum for a message object of type '<RunVelocityTest-request>"
  "0da28cc5facb55eb49569bd980fe316c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunVelocityTest-request)))
  "Returns md5sum for a message object of type 'RunVelocityTest-request"
  "0da28cc5facb55eb49569bd980fe316c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunVelocityTest-request>)))
  "Returns full string definition for message of type '<RunVelocityTest-request>"
  (cl:format cl:nil "uint32 time_ms # Test time in milliseconds.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunVelocityTest-request)))
  "Returns full string definition for message of type 'RunVelocityTest-request"
  (cl:format cl:nil "uint32 time_ms # Test time in milliseconds.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunVelocityTest-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunVelocityTest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunVelocityTest-request
    (cl:cons ':time_ms (time_ms msg))
))
;//! \htmlinclude RunVelocityTest-response.msg.html

(cl:defclass <RunVelocityTest-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass RunVelocityTest-response (<RunVelocityTest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunVelocityTest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunVelocityTest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<RunVelocityTest-response> is deprecated: use statek_hw-srv:RunVelocityTest-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RunVelocityTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:success-val is deprecated.  Use statek_hw-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <RunVelocityTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:velocity-val is deprecated.  Use statek_hw-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunVelocityTest-response>) ostream)
  "Serializes a message object of type '<RunVelocityTest-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunVelocityTest-response>) istream)
  "Deserializes a message object of type '<RunVelocityTest-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunVelocityTest-response>)))
  "Returns string type for a service object of type '<RunVelocityTest-response>"
  "statek_hw/RunVelocityTestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunVelocityTest-response)))
  "Returns string type for a service object of type 'RunVelocityTest-response"
  "statek_hw/RunVelocityTestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunVelocityTest-response>)))
  "Returns md5sum for a message object of type '<RunVelocityTest-response>"
  "0da28cc5facb55eb49569bd980fe316c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunVelocityTest-response)))
  "Returns md5sum for a message object of type 'RunVelocityTest-response"
  "0da28cc5facb55eb49569bd980fe316c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunVelocityTest-response>)))
  "Returns full string definition for message of type '<RunVelocityTest-response>"
  (cl:format cl:nil "bool success # Set to true on success~%float64 velocity # Found velocity in rad/s.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunVelocityTest-response)))
  "Returns full string definition for message of type 'RunVelocityTest-response"
  (cl:format cl:nil "bool success # Set to true on success~%float64 velocity # Found velocity in rad/s.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunVelocityTest-response>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunVelocityTest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunVelocityTest-response
    (cl:cons ':success (success msg))
    (cl:cons ':velocity (velocity msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunVelocityTest)))
  'RunVelocityTest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunVelocityTest)))
  'RunVelocityTest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunVelocityTest)))
  "Returns string type for a service object of type '<RunVelocityTest>"
  "statek_hw/RunVelocityTest")