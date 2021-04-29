; Auto-generated. Do not edit!


(cl:in-package statek_hw-srv)


;//! \htmlinclude RunModelIdentification-request.msg.html

(cl:defclass <RunModelIdentification-request> (roslisp-msg-protocol:ros-message)
  ((identification_time_ms
    :reader identification_time_ms
    :initarg :identification_time_ms
    :type cl:integer
    :initform 0))
)

(cl:defclass RunModelIdentification-request (<RunModelIdentification-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunModelIdentification-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunModelIdentification-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<RunModelIdentification-request> is deprecated: use statek_hw-srv:RunModelIdentification-request instead.")))

(cl:ensure-generic-function 'identification_time_ms-val :lambda-list '(m))
(cl:defmethod identification_time_ms-val ((m <RunModelIdentification-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:identification_time_ms-val is deprecated.  Use statek_hw-srv:identification_time_ms instead.")
  (identification_time_ms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunModelIdentification-request>) ostream)
  "Serializes a message object of type '<RunModelIdentification-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identification_time_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identification_time_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identification_time_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identification_time_ms)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunModelIdentification-request>) istream)
  "Deserializes a message object of type '<RunModelIdentification-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identification_time_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identification_time_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identification_time_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identification_time_ms)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunModelIdentification-request>)))
  "Returns string type for a service object of type '<RunModelIdentification-request>"
  "statek_hw/RunModelIdentificationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunModelIdentification-request)))
  "Returns string type for a service object of type 'RunModelIdentification-request"
  "statek_hw/RunModelIdentificationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunModelIdentification-request>)))
  "Returns md5sum for a message object of type '<RunModelIdentification-request>"
  "d0062710edaa1716482716b90a44c970")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunModelIdentification-request)))
  "Returns md5sum for a message object of type 'RunModelIdentification-request"
  "d0062710edaa1716482716b90a44c970")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunModelIdentification-request>)))
  "Returns full string definition for message of type '<RunModelIdentification-request>"
  (cl:format cl:nil "uint32 identification_time_ms~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunModelIdentification-request)))
  "Returns full string definition for message of type 'RunModelIdentification-request"
  (cl:format cl:nil "uint32 identification_time_ms~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunModelIdentification-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunModelIdentification-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunModelIdentification-request
    (cl:cons ':identification_time_ms (identification_time_ms msg))
))
;//! \htmlinclude RunModelIdentification-response.msg.html

(cl:defclass <RunModelIdentification-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (sampling_time
    :reader sampling_time
    :initarg :sampling_time
    :type cl:float
    :initform 0.0)
   (samples
    :reader samples
    :initarg :samples
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RunModelIdentification-response (<RunModelIdentification-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunModelIdentification-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunModelIdentification-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name statek_hw-srv:<RunModelIdentification-response> is deprecated: use statek_hw-srv:RunModelIdentification-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RunModelIdentification-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:success-val is deprecated.  Use statek_hw-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'sampling_time-val :lambda-list '(m))
(cl:defmethod sampling_time-val ((m <RunModelIdentification-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:sampling_time-val is deprecated.  Use statek_hw-srv:sampling_time instead.")
  (sampling_time m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <RunModelIdentification-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader statek_hw-srv:samples-val is deprecated.  Use statek_hw-srv:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunModelIdentification-response>) ostream)
  "Serializes a message object of type '<RunModelIdentification-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sampling_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'samples))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunModelIdentification-response>) istream)
  "Deserializes a message object of type '<RunModelIdentification-response>"
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
    (cl:setf (cl:slot-value msg 'sampling_time) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'samples) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'samples)))
    (cl:dotimes (i 100)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunModelIdentification-response>)))
  "Returns string type for a service object of type '<RunModelIdentification-response>"
  "statek_hw/RunModelIdentificationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunModelIdentification-response)))
  "Returns string type for a service object of type 'RunModelIdentification-response"
  "statek_hw/RunModelIdentificationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunModelIdentification-response>)))
  "Returns md5sum for a message object of type '<RunModelIdentification-response>"
  "d0062710edaa1716482716b90a44c970")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunModelIdentification-response)))
  "Returns md5sum for a message object of type 'RunModelIdentification-response"
  "d0062710edaa1716482716b90a44c970")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunModelIdentification-response>)))
  "Returns full string definition for message of type '<RunModelIdentification-response>"
  (cl:format cl:nil "bool success~%float64 sampling_time~%float64[100] samples~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunModelIdentification-response)))
  "Returns full string definition for message of type 'RunModelIdentification-response"
  (cl:format cl:nil "bool success~%float64 sampling_time~%float64[100] samples~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunModelIdentification-response>))
  (cl:+ 0
     1
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunModelIdentification-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunModelIdentification-response
    (cl:cons ':success (success msg))
    (cl:cons ':sampling_time (sampling_time msg))
    (cl:cons ':samples (samples msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunModelIdentification)))
  'RunModelIdentification-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunModelIdentification)))
  'RunModelIdentification-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunModelIdentification)))
  "Returns string type for a service object of type '<RunModelIdentification>"
  "statek_hw/RunModelIdentification")