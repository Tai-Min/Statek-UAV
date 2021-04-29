; Auto-generated. Do not edit!


(cl:in-package statek_hw-srv)


;//! \htmlinclude SetOdomParams-request.msg.html

(cl:defclass <SetOdomParams-request> (roslisp-msg-protocol:ros-message)
  ((odom_update_rate_ms
    :reader odom_update_rate_ms
    :initarg :odom_update_rate_ms
    :type cl:integer
    :initform 0)
   (distance_between_wheels
    :reader distance_between_wheels
    :initarg :distance_between_wheels
    :type cl:float
    :initform 0.0)
   (wheel_radius
    :reader wheel_radius
    :initarg :wheel_radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetOdomParams-request (<SetOdomParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOdomParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOdomParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<SetOdomParams-request> is deprecated: use statek_hw-srv:SetOdomParams-request instead.")))

(cl:ensure-generic-function 'odom_update_rate_ms-val :lambda-list '(m))
(cl:defmethod odom_update_rate_ms-val ((m <SetOdomParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:odom_update_rate_ms-val is deprecated.  Use statek_hw-srv:odom_update_rate_ms instead.")
  (odom_update_rate_ms m))

(cl:ensure-generic-function 'distance_between_wheels-val :lambda-list '(m))
(cl:defmethod distance_between_wheels-val ((m <SetOdomParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:distance_between_wheels-val is deprecated.  Use statek_hw-srv:distance_between_wheels instead.")
  (distance_between_wheels m))

(cl:ensure-generic-function 'wheel_radius-val :lambda-list '(m))
(cl:defmethod wheel_radius-val ((m <SetOdomParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:wheel_radius-val is deprecated.  Use statek_hw-srv:wheel_radius instead.")
  (wheel_radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOdomParams-request>) ostream)
  "Serializes a message object of type '<SetOdomParams-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'odom_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'odom_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'odom_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'odom_update_rate_ms)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance_between_wheels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wheel_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOdomParams-request>) istream)
  "Deserializes a message object of type '<SetOdomParams-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'odom_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'odom_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'odom_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'odom_update_rate_ms)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_between_wheels) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOdomParams-request>)))
  "Returns string type for a service object of type '<SetOdomParams-request>"
  "statek_hw/SetOdomParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOdomParams-request)))
  "Returns string type for a service object of type 'SetOdomParams-request"
  "statek_hw/SetOdomParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOdomParams-request>)))
  "Returns md5sum for a message object of type '<SetOdomParams-request>"
  "1c8fcad49bde18e0cd305a59dcc09b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOdomParams-request)))
  "Returns md5sum for a message object of type 'SetOdomParams-request"
  "1c8fcad49bde18e0cd305a59dcc09b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOdomParams-request>)))
  "Returns full string definition for message of type '<SetOdomParams-request>"
  (cl:format cl:nil "uint32 odom_update_rate_ms # Set to 0 to ignore.~%float64 distance_between_wheels # Set to negative to ignore.~%float64 wheel_radius # Set to negative to ignore.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOdomParams-request)))
  "Returns full string definition for message of type 'SetOdomParams-request"
  (cl:format cl:nil "uint32 odom_update_rate_ms # Set to 0 to ignore.~%float64 distance_between_wheels # Set to negative to ignore.~%float64 wheel_radius # Set to negative to ignore.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOdomParams-request>))
  (cl:+ 0
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOdomParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOdomParams-request
    (cl:cons ':odom_update_rate_ms (odom_update_rate_ms msg))
    (cl:cons ':distance_between_wheels (distance_between_wheels msg))
    (cl:cons ':wheel_radius (wheel_radius msg))
))
;//! \htmlinclude SetOdomParams-response.msg.html

(cl:defclass <SetOdomParams-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetOdomParams-response (<SetOdomParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOdomParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOdomParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<SetOdomParams-response> is deprecated: use statek_hw-srv:SetOdomParams-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetOdomParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:success-val is deprecated.  Use statek_hw-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOdomParams-response>) ostream)
  "Serializes a message object of type '<SetOdomParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOdomParams-response>) istream)
  "Deserializes a message object of type '<SetOdomParams-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOdomParams-response>)))
  "Returns string type for a service object of type '<SetOdomParams-response>"
  "statek_hw/SetOdomParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOdomParams-response)))
  "Returns string type for a service object of type 'SetOdomParams-response"
  "statek_hw/SetOdomParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOdomParams-response>)))
  "Returns md5sum for a message object of type '<SetOdomParams-response>"
  "1c8fcad49bde18e0cd305a59dcc09b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOdomParams-response)))
  "Returns md5sum for a message object of type 'SetOdomParams-response"
  "1c8fcad49bde18e0cd305a59dcc09b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOdomParams-response>)))
  "Returns full string definition for message of type '<SetOdomParams-response>"
  (cl:format cl:nil "bool success # Set to true on success.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOdomParams-response)))
  "Returns full string definition for message of type 'SetOdomParams-response"
  (cl:format cl:nil "bool success # Set to true on success.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOdomParams-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOdomParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOdomParams-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOdomParams)))
  'SetOdomParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOdomParams)))
  'SetOdomParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOdomParams)))
  "Returns string type for a service object of type '<SetOdomParams>"
  "statek_hw/SetOdomParams")