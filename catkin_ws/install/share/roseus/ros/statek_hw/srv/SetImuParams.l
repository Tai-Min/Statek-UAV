;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::SetImuParams)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'SetImuParams (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::SETIMUPARAMS")
  (make-package "STATEK_HW::SETIMUPARAMS"))
(unless (find-package "STATEK_HW::SETIMUPARAMSREQUEST")
  (make-package "STATEK_HW::SETIMUPARAMSREQUEST"))
(unless (find-package "STATEK_HW::SETIMUPARAMSRESPONSE")
  (make-package "STATEK_HW::SETIMUPARAMSRESPONSE"))

(in-package "ROS")





(defclass statek_hw::SetImuParamsRequest
  :super ros::object
  :slots (_imu_update_rate_ms _acc_bias _gyro_bias _mag_bias _mag_scale _mag_dec_deg _mag_dec_min _mag_dec_sec ))

(defmethod statek_hw::SetImuParamsRequest
  (:init
   (&key
    ((:imu_update_rate_ms __imu_update_rate_ms) 0)
    ((:acc_bias __acc_bias) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:gyro_bias __gyro_bias) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mag_bias __mag_bias) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mag_scale __mag_scale) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mag_dec_deg __mag_dec_deg) 0.0)
    ((:mag_dec_min __mag_dec_min) 0.0)
    ((:mag_dec_sec __mag_dec_sec) 0.0)
    )
   (send-super :init)
   (setq _imu_update_rate_ms (round __imu_update_rate_ms))
   (setq _acc_bias __acc_bias)
   (setq _gyro_bias __gyro_bias)
   (setq _mag_bias __mag_bias)
   (setq _mag_scale __mag_scale)
   (setq _mag_dec_deg (float __mag_dec_deg))
   (setq _mag_dec_min (float __mag_dec_min))
   (setq _mag_dec_sec (float __mag_dec_sec))
   self)
  (:imu_update_rate_ms
   (&optional __imu_update_rate_ms)
   (if __imu_update_rate_ms (setq _imu_update_rate_ms __imu_update_rate_ms)) _imu_update_rate_ms)
  (:acc_bias
   (&optional __acc_bias)
   (if __acc_bias (setq _acc_bias __acc_bias)) _acc_bias)
  (:gyro_bias
   (&optional __gyro_bias)
   (if __gyro_bias (setq _gyro_bias __gyro_bias)) _gyro_bias)
  (:mag_bias
   (&optional __mag_bias)
   (if __mag_bias (setq _mag_bias __mag_bias)) _mag_bias)
  (:mag_scale
   (&optional __mag_scale)
   (if __mag_scale (setq _mag_scale __mag_scale)) _mag_scale)
  (:mag_dec_deg
   (&optional __mag_dec_deg)
   (if __mag_dec_deg (setq _mag_dec_deg __mag_dec_deg)) _mag_dec_deg)
  (:mag_dec_min
   (&optional __mag_dec_min)
   (if __mag_dec_min (setq _mag_dec_min __mag_dec_min)) _mag_dec_min)
  (:mag_dec_sec
   (&optional __mag_dec_sec)
   (if __mag_dec_sec (setq _mag_dec_sec __mag_dec_sec)) _mag_dec_sec)
  (:serialization-length
   ()
   (+
    ;; uint32 _imu_update_rate_ms
    4
    ;; float64[3] _acc_bias
    (* 8    3)
    ;; float64[3] _gyro_bias
    (* 8    3)
    ;; float64[3] _mag_bias
    (* 8    3)
    ;; float64[3] _mag_scale
    (* 8    3)
    ;; float64 _mag_dec_deg
    8
    ;; float64 _mag_dec_min
    8
    ;; float64 _mag_dec_sec
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _imu_update_rate_ms
       (write-long _imu_update_rate_ms s)
     ;; float64[3] _acc_bias
     (dotimes (i 3)
       (sys::poke (elt _acc_bias i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _gyro_bias
     (dotimes (i 3)
       (sys::poke (elt _gyro_bias i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _mag_bias
     (dotimes (i 3)
       (sys::poke (elt _mag_bias i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _mag_scale
     (dotimes (i 3)
       (sys::poke (elt _mag_scale i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _mag_dec_deg
       (sys::poke _mag_dec_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mag_dec_min
       (sys::poke _mag_dec_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mag_dec_sec
       (sys::poke _mag_dec_sec (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _imu_update_rate_ms
     (setq _imu_update_rate_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[3] _acc_bias
   (dotimes (i (length _acc_bias))
     (setf (elt _acc_bias i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _gyro_bias
   (dotimes (i (length _gyro_bias))
     (setf (elt _gyro_bias i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _mag_bias
   (dotimes (i (length _mag_bias))
     (setf (elt _mag_bias i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _mag_scale
   (dotimes (i (length _mag_scale))
     (setf (elt _mag_scale i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _mag_dec_deg
     (setq _mag_dec_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mag_dec_min
     (setq _mag_dec_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mag_dec_sec
     (setq _mag_dec_sec (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass statek_hw::SetImuParamsResponse
  :super ros::object
  :slots (_success ))

(defmethod statek_hw::SetImuParamsResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass statek_hw::SetImuParams
  :super ros::object
  :slots ())

(setf (get statek_hw::SetImuParams :md5sum-) "fc6fb75dba4829e04baa420f42b24adc")
(setf (get statek_hw::SetImuParams :datatype-) "statek_hw/SetImuParams")
(setf (get statek_hw::SetImuParams :request) statek_hw::SetImuParamsRequest)
(setf (get statek_hw::SetImuParams :response) statek_hw::SetImuParamsResponse)

(defmethod statek_hw::SetImuParamsRequest
  (:response () (instance statek_hw::SetImuParamsResponse :init)))

(setf (get statek_hw::SetImuParamsRequest :md5sum-) "fc6fb75dba4829e04baa420f42b24adc")
(setf (get statek_hw::SetImuParamsRequest :datatype-) "statek_hw/SetImuParamsRequest")
(setf (get statek_hw::SetImuParamsRequest :definition-)
      "uint32 imu_update_rate_ms
float64[3] acc_bias # Some value from calibrated MPU9250.
float64[3] gyro_bias # Some value from calibrated MPU9250.
float64[3] mag_bias # Some value from calibrated MPU9250.
float64[3] mag_scale # Some value from calibrated MPU9250.
float64 mag_dec_deg # From internet or whatever.
float64 mag_dec_min
float64 mag_dec_sec
---
bool success # Set to true on success.
")

(setf (get statek_hw::SetImuParamsResponse :md5sum-) "fc6fb75dba4829e04baa420f42b24adc")
(setf (get statek_hw::SetImuParamsResponse :datatype-) "statek_hw/SetImuParamsResponse")
(setf (get statek_hw::SetImuParamsResponse :definition-)
      "uint32 imu_update_rate_ms
float64[3] acc_bias # Some value from calibrated MPU9250.
float64[3] gyro_bias # Some value from calibrated MPU9250.
float64[3] mag_bias # Some value from calibrated MPU9250.
float64[3] mag_scale # Some value from calibrated MPU9250.
float64 mag_dec_deg # From internet or whatever.
float64 mag_dec_min
float64 mag_dec_sec
---
bool success # Set to true on success.
")



(provide :statek_hw/SetImuParams "fc6fb75dba4829e04baa420f42b24adc")

