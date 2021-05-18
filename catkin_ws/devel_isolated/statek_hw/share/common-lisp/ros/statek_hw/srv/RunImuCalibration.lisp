; Auto-generated. Do not edit!


(cl:in-package statek_hw-srv)


;//! \htmlinclude RunImuCalibration-request.msg.html

(cl:defclass <RunImuCalibration-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RunImuCalibration-request (<RunImuCalibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunImuCalibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunImuCalibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<RunImuCalibration-request> is deprecated: use statek_hw-srv:RunImuCalibration-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunImuCalibration-request>) ostream)
  "Serializes a message object of type '<RunImuCalibration-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunImuCalibration-request>) istream)
  "Deserializes a message object of type '<RunImuCalibration-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunImuCalibration-request>)))
  "Returns string type for a service object of type '<RunImuCalibration-request>"
  "statek_hw/RunImuCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunImuCalibration-request)))
  "Returns string type for a service object of type 'RunImuCalibration-request"
  "statek_hw/RunImuCalibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunImuCalibration-request>)))
  "Returns md5sum for a message object of type '<RunImuCalibration-request>"
  "6cd60c1db4de1bdcd039c0b56b7d1d09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunImuCalibration-request)))
  "Returns md5sum for a message object of type 'RunImuCalibration-request"
  "6cd60c1db4de1bdcd039c0b56b7d1d09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunImuCalibration-request>)))
  "Returns full string definition for message of type '<RunImuCalibration-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunImuCalibration-request)))
  "Returns full string definition for message of type 'RunImuCalibration-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunImuCalibration-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunImuCalibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunImuCalibration-request
))
;//! \htmlinclude RunImuCalibration-response.msg.html

(cl:defclass <RunImuCalibration-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
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
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RunImuCalibration-response (<RunImuCalibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunImuCalibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunImuCalibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<RunImuCalibration-response> is deprecated: use statek_hw-srv:RunImuCalibration-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RunImuCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:success-val is deprecated.  Use statek_hw-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'acc_bias-val :lambda-list '(m))
(cl:defmethod acc_bias-val ((m <RunImuCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:acc_bias-val is deprecated.  Use statek_hw-srv:acc_bias instead.")
  (acc_bias m))

(cl:ensure-generic-function 'gyro_bias-val :lambda-list '(m))
(cl:defmethod gyro_bias-val ((m <RunImuCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:gyro_bias-val is deprecated.  Use statek_hw-srv:gyro_bias instead.")
  (gyro_bias m))

(cl:ensure-generic-function 'mag_bias-val :lambda-list '(m))
(cl:defmethod mag_bias-val ((m <RunImuCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:mag_bias-val is deprecated.  Use statek_hw-srv:mag_bias instead.")
  (mag_bias m))

(cl:ensure-generic-function 'mag_scale-val :lambda-list '(m))
(cl:defmethod mag_scale-val ((m <RunImuCalibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:mag_scale-val is deprecated.  Use statek_hw-srv:mag_scale instead.")
  (mag_scale m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunImuCalibration-response>) ostream)
  "Serializes a message object of type '<RunImuCalibration-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunImuCalibration-response>) istream)
  "Deserializes a message object of type '<RunImuCalibration-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunImuCalibration-response>)))
  "Returns string type for a service object of type '<RunImuCalibration-response>"
  "statek_hw/RunImuCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunImuCalibration-response)))
  "Returns string type for a service object of type 'RunImuCalibration-response"
  "statek_hw/RunImuCalibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunImuCalibration-response>)))
  "Returns md5sum for a message object of type '<RunImuCalibration-response>"
  "6cd60c1db4de1bdcd039c0b56b7d1d09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunImuCalibration-response)))
  "Returns md5sum for a message object of type 'RunImuCalibration-response"
  "6cd60c1db4de1bdcd039c0b56b7d1d09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunImuCalibration-response>)))
  "Returns full string definition for message of type '<RunImuCalibration-response>"
  (cl:format cl:nil "bool success # Set to true on success~%float64[3] acc_bias # Some value from calibrated MPU9250.~%float64[3] gyro_bias # Some value from calibrated MPU9250.~%float64[3] mag_bias # Some value from calibrated MPU9250.~%float64[3] mag_scale # Some value from calibrated MPU9250.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunImuCalibration-response)))
  "Returns full string definition for message of type 'RunImuCalibration-response"
  (cl:format cl:nil "bool success # Set to true on success~%float64[3] acc_bias # Some value from calibrated MPU9250.~%float64[3] gyro_bias # Some value from calibrated MPU9250.~%float64[3] mag_bias # Some value from calibrated MPU9250.~%float64[3] mag_scale # Some value from calibrated MPU9250.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunImuCalibration-response>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc_bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyro_bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mag_scale) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunImuCalibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunImuCalibration-response
    (cl:cons ':success (success msg))
    (cl:cons ':acc_bias (acc_bias msg))
    (cl:cons ':gyro_bias (gyro_bias msg))
    (cl:cons ':mag_bias (mag_bias msg))
    (cl:cons ':mag_scale (mag_scale msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunImuCalibration)))
  'RunImuCalibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunImuCalibration)))
  'RunImuCalibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunImuCalibration)))
  "Returns string type for a service object of type '<RunImuCalibration>"
  "statek_hw/RunImuCalibration")