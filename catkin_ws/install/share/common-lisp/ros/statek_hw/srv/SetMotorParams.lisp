; Auto-generated. Do not edit!


(cl:in-package statek_hw-srv)


;//! \htmlinclude SetMotorParams-request.msg.html

(cl:defclass <SetMotorParams-request> (roslisp-msg-protocol:ros-message)
  ((loop_update_rate_ms
    :reader loop_update_rate_ms
    :initarg :loop_update_rate_ms
    :type cl:integer
    :initform 0)
   (wheel_max_angular_velocity
    :reader wheel_max_angular_velocity
    :initarg :wheel_max_angular_velocity
    :type cl:float
    :initform 0.0)
   (smoothing_factor
    :reader smoothing_factor
    :initarg :smoothing_factor
    :type cl:float
    :initform 0.0)
   (kp
    :reader kp
    :initarg :kp
    :type cl:float
    :initform 0.0)
   (ki
    :reader ki
    :initarg :ki
    :type cl:float
    :initform 0.0)
   (kd
    :reader kd
    :initarg :kd
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetMotorParams-request (<SetMotorParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMotorParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMotorParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<SetMotorParams-request> is deprecated: use statek_hw-srv:SetMotorParams-request instead.")))

(cl:ensure-generic-function 'loop_update_rate_ms-val :lambda-list '(m))
(cl:defmethod loop_update_rate_ms-val ((m <SetMotorParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:loop_update_rate_ms-val is deprecated.  Use statek_hw-srv:loop_update_rate_ms instead.")
  (loop_update_rate_ms m))

(cl:ensure-generic-function 'wheel_max_angular_velocity-val :lambda-list '(m))
(cl:defmethod wheel_max_angular_velocity-val ((m <SetMotorParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:wheel_max_angular_velocity-val is deprecated.  Use statek_hw-srv:wheel_max_angular_velocity instead.")
  (wheel_max_angular_velocity m))

(cl:ensure-generic-function 'smoothing_factor-val :lambda-list '(m))
(cl:defmethod smoothing_factor-val ((m <SetMotorParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:smoothing_factor-val is deprecated.  Use statek_hw-srv:smoothing_factor instead.")
  (smoothing_factor m))

(cl:ensure-generic-function 'kp-val :lambda-list '(m))
(cl:defmethod kp-val ((m <SetMotorParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:kp-val is deprecated.  Use statek_hw-srv:kp instead.")
  (kp m))

(cl:ensure-generic-function 'ki-val :lambda-list '(m))
(cl:defmethod ki-val ((m <SetMotorParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:ki-val is deprecated.  Use statek_hw-srv:ki instead.")
  (ki m))

(cl:ensure-generic-function 'kd-val :lambda-list '(m))
(cl:defmethod kd-val ((m <SetMotorParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:kd-val is deprecated.  Use statek_hw-srv:kd instead.")
  (kd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMotorParams-request>) ostream)
  "Serializes a message object of type '<SetMotorParams-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_update_rate_ms)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wheel_max_angular_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'smoothing_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMotorParams-request>) istream)
  "Deserializes a message object of type '<SetMotorParams-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'loop_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'loop_update_rate_ms)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_max_angular_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'smoothing_factor) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ki) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMotorParams-request>)))
  "Returns string type for a service object of type '<SetMotorParams-request>"
  "statek_hw/SetMotorParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotorParams-request)))
  "Returns string type for a service object of type 'SetMotorParams-request"
  "statek_hw/SetMotorParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMotorParams-request>)))
  "Returns md5sum for a message object of type '<SetMotorParams-request>"
  "f3ac856c8e72619c8db00906cbcd4cd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMotorParams-request)))
  "Returns md5sum for a message object of type 'SetMotorParams-request"
  "f3ac856c8e72619c8db00906cbcd4cd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMotorParams-request>)))
  "Returns full string definition for message of type '<SetMotorParams-request>"
  (cl:format cl:nil "uint32 loop_update_rate_ms # In ms. Set to 0 to ignore.~%float64 wheel_max_angular_velocity # In rad/s. Set to negative to ignore.~%float64 smoothing_factor # Between 0 and 1. Set to negative to ignore.~%float64 kp # Set to negative to ignore.~%float64 ki # Set to negative to ignore.~%float64 kd # Set to negative to ignore.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMotorParams-request)))
  "Returns full string definition for message of type 'SetMotorParams-request"
  (cl:format cl:nil "uint32 loop_update_rate_ms # In ms. Set to 0 to ignore.~%float64 wheel_max_angular_velocity # In rad/s. Set to negative to ignore.~%float64 smoothing_factor # Between 0 and 1. Set to negative to ignore.~%float64 kp # Set to negative to ignore.~%float64 ki # Set to negative to ignore.~%float64 kd # Set to negative to ignore.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMotorParams-request>))
  (cl:+ 0
     4
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMotorParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMotorParams-request
    (cl:cons ':loop_update_rate_ms (loop_update_rate_ms msg))
    (cl:cons ':wheel_max_angular_velocity (wheel_max_angular_velocity msg))
    (cl:cons ':smoothing_factor (smoothing_factor msg))
    (cl:cons ':kp (kp msg))
    (cl:cons ':ki (ki msg))
    (cl:cons ':kd (kd msg))
))
;//! \htmlinclude SetMotorParams-response.msg.html

(cl:defclass <SetMotorParams-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetMotorParams-response (<SetMotorParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMotorParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMotorParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<SetMotorParams-response> is deprecated: use statek_hw-srv:SetMotorParams-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetMotorParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:success-val is deprecated.  Use statek_hw-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMotorParams-response>) ostream)
  "Serializes a message object of type '<SetMotorParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMotorParams-response>) istream)
  "Deserializes a message object of type '<SetMotorParams-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMotorParams-response>)))
  "Returns string type for a service object of type '<SetMotorParams-response>"
  "statek_hw/SetMotorParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotorParams-response)))
  "Returns string type for a service object of type 'SetMotorParams-response"
  "statek_hw/SetMotorParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMotorParams-response>)))
  "Returns md5sum for a message object of type '<SetMotorParams-response>"
  "f3ac856c8e72619c8db00906cbcd4cd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMotorParams-response)))
  "Returns md5sum for a message object of type 'SetMotorParams-response"
  "f3ac856c8e72619c8db00906cbcd4cd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMotorParams-response>)))
  "Returns full string definition for message of type '<SetMotorParams-response>"
  (cl:format cl:nil "bool success # Set to true on success.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMotorParams-response)))
  "Returns full string definition for message of type 'SetMotorParams-response"
  (cl:format cl:nil "bool success # Set to true on success.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMotorParams-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMotorParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMotorParams-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMotorParams)))
  'SetMotorParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMotorParams)))
  'SetMotorParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotorParams)))
  "Returns string type for a service object of type '<SetMotorParams>"
  "statek_hw/SetMotorParams")