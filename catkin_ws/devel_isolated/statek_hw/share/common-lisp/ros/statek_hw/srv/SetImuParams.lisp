; Auto-generated. Do not edit!


(cl:in-package statek_hw-srv)


;//! \htmlinclude SetImuParams-request.msg.html

(cl:defclass <SetImuParams-request> (roslisp-msg-protocol:ros-message)
  ((imu_update_rate_ms
    :reader imu_update_rate_ms
    :initarg :imu_update_rate_ms
    :type cl:integer
    :initform 0)
   (acc_bias
    :reader acc_bias
    :initarg :acc_bias
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (gyro_bias
    :reader gyro_bias
    :initarg :gyro_bias
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mag_bias
    :reader mag_bias
    :initarg :mag_bias
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mag_scale
    :reader mag_scale
    :initarg :mag_scale
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mag_dec
    :reader mag_dec
    :initarg :mag_dec
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SetImuParams-request (<SetImuParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetImuParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetImuParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<SetImuParams-request> is deprecated: use statek_hw-srv:SetImuParams-request instead.")))

(cl:ensure-generic-function 'imu_update_rate_ms-val :lambda-list '(m))
(cl:defmethod imu_update_rate_ms-val ((m <SetImuParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:imu_update_rate_ms-val is deprecated.  Use statek_hw-srv:imu_update_rate_ms instead.")
  (imu_update_rate_ms m))

(cl:ensure-generic-function 'acc_bias-val :lambda-list '(m))
(cl:defmethod acc_bias-val ((m <SetImuParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:acc_bias-val is deprecated.  Use statek_hw-srv:acc_bias instead.")
  (acc_bias m))

(cl:ensure-generic-function 'gyro_bias-val :lambda-list '(m))
(cl:defmethod gyro_bias-val ((m <SetImuParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:gyro_bias-val is deprecated.  Use statek_hw-srv:gyro_bias instead.")
  (gyro_bias m))

(cl:ensure-generic-function 'mag_bias-val :lambda-list '(m))
(cl:defmethod mag_bias-val ((m <SetImuParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:mag_bias-val is deprecated.  Use statek_hw-srv:mag_bias instead.")
  (mag_bias m))

(cl:ensure-generic-function 'mag_scale-val :lambda-list '(m))
(cl:defmethod mag_scale-val ((m <SetImuParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:mag_scale-val is deprecated.  Use statek_hw-srv:mag_scale instead.")
  (mag_scale m))

(cl:ensure-generic-function 'mag_dec-val :lambda-list '(m))
(cl:defmethod mag_dec-val ((m <SetImuParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:mag_dec-val is deprecated.  Use statek_hw-srv:mag_dec instead.")
  (mag_dec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetImuParams-request>) ostream)
  "Serializes a message object of type '<SetImuParams-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'imu_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'imu_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'imu_update_rate_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'imu_update_rate_ms)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'acc_bias))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'gyro_bias))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'mag_bias))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'mag_scale))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mag_dec))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetImuParams-request>) istream)
  "Deserializes a message object of type '<SetImuParams-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'imu_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'imu_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'imu_update_rate_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'imu_update_rate_ms)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'acc_bias) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'acc_bias)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'gyro_bias) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyro_bias)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'mag_bias) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_bias)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'mag_scale) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_scale)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'mag_dec) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mag_dec)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetImuParams-request>)))
  "Returns string type for a service object of type '<SetImuParams-request>"
  "statek_hw/SetImuParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetImuParams-request)))
  "Returns string type for a service object of type 'SetImuParams-request"
  "statek_hw/SetImuParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetImuParams-request>)))
  "Returns md5sum for a message object of type '<SetImuParams-request>"
  "20fdbb57165b3957921339533f874eaf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetImuParams-request)))
  "Returns md5sum for a message object of type 'SetImuParams-request"
  "20fdbb57165b3957921339533f874eaf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetImuParams-request>)))
  "Returns full string definition for message of type '<SetImuParams-request>"
  (cl:format cl:nil "uint32 imu_update_rate_ms~%float64[3] acc_bias # Some value from calibrated MPU9250.~%float64[3] gyro_bias # Some value from calibrated MPU9250.~%float64[3] mag_bias # Some value from calibrated MPU9250.~%float64[3] mag_scale # Some value from calibrated MPU9250.~%int16[3] mag_dec # Deg, min, sec.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetImuParams-request)))
  "Returns full string definition for message of type 'SetImuParams-request"
  (cl:format cl:nil "uint32 imu_update_rate_ms~%float64[3] acc_bias # Some value from calibrated MPU9250.~%float64[3] gyro_bias # Some value from calibrated MPU9250.~%float64[3] mag_bias # Some value from calibrated MPU9250.~%float64[3] mag_scale # Some value from calibrated MPU9250.~%int16[3] mag_dec # Deg, min, sec.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetImuParams-request>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyro_bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_scale) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_dec) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetImuParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetImuParams-request
    (cl:cons ':imu_update_rate_ms (imu_update_rate_ms msg))
    (cl:cons ':acc_bias (acc_bias msg))
    (cl:cons ':gyro_bias (gyro_bias msg))
    (cl:cons ':mag_bias (mag_bias msg))
    (cl:cons ':mag_scale (mag_scale msg))
    (cl:cons ':mag_dec (mag_dec msg))
))
;//! \htmlinclude SetImuParams-response.msg.html

(cl:defclass <SetImuParams-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetImuParams-response (<SetImuParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetImuParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetImuParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<SetImuParams-response> is deprecated: use statek_hw-srv:SetImuParams-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetImuParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:success-val is deprecated.  Use statek_hw-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetImuParams-response>) ostream)
  "Serializes a message object of type '<SetImuParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetImuParams-response>) istream)
  "Deserializes a message object of type '<SetImuParams-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetImuParams-response>)))
  "Returns string type for a service object of type '<SetImuParams-response>"
  "statek_hw/SetImuParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetImuParams-response)))
  "Returns string type for a service object of type 'SetImuParams-response"
  "statek_hw/SetImuParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetImuParams-response>)))
  "Returns md5sum for a message object of type '<SetImuParams-response>"
  "20fdbb57165b3957921339533f874eaf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetImuParams-response)))
  "Returns md5sum for a message object of type 'SetImuParams-response"
  "20fdbb57165b3957921339533f874eaf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetImuParams-response>)))
  "Returns full string definition for message of type '<SetImuParams-response>"
  (cl:format cl:nil "bool success # Set to true on success.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetImuParams-response)))
  "Returns full string definition for message of type 'SetImuParams-response"
  (cl:format cl:nil "bool success # Set to true on success.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetImuParams-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetImuParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetImuParams-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetImuParams)))
  'SetImuParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetImuParams)))
  'SetImuParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetImuParams)))
  "Returns string type for a service object of type '<SetImuParams>"
  "statek_hw/SetImuParams")