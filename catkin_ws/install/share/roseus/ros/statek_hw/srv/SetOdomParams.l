;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::SetOdomParams)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'SetOdomParams (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::SETODOMPARAMS")
  (make-package "STATEK_HW::SETODOMPARAMS"))
(unless (find-package "STATEK_HW::SETODOMPARAMSREQUEST")
  (make-package "STATEK_HW::SETODOMPARAMSREQUEST"))
(unless (find-package "STATEK_HW::SETODOMPARAMSRESPONSE")
  (make-package "STATEK_HW::SETODOMPARAMSRESPONSE"))

(in-package "ROS")





(defclass statek_hw::SetOdomParamsRequest
  :super ros::object
  :slots (_odom_update_rate_ms _distance_between_wheels _wheel_radius ))

(defmethod statek_hw::SetOdomParamsRequest
  (:init
   (&key
    ((:odom_update_rate_ms __odom_update_rate_ms) 0)
    ((:distance_between_wheels __distance_between_wheels) 0.0)
    ((:wheel_radius __wheel_radius) 0.0)
    )
   (send-super :init)
   (setq _odom_update_rate_ms (round __odom_update_rate_ms))
   (setq _distance_between_wheels (float __distance_between_wheels))
   (setq _wheel_radius (float __wheel_radius))
   self)
  (:odom_update_rate_ms
   (&optional __odom_update_rate_ms)
   (if __odom_update_rate_ms (setq _odom_update_rate_ms __odom_update_rate_ms)) _odom_update_rate_ms)
  (:distance_between_wheels
   (&optional __distance_between_wheels)
   (if __distance_between_wheels (setq _distance_between_wheels __distance_between_wheels)) _distance_between_wheels)
  (:wheel_radius
   (&optional __wheel_radius)
   (if __wheel_radius (setq _wheel_radius __wheel_radius)) _wheel_radius)
  (:serialization-length
   ()
   (+
    ;; uint32 _odom_update_rate_ms
    4
    ;; float64 _distance_between_wheels
    8
    ;; float64 _wheel_radius
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _odom_update_rate_ms
       (write-long _odom_update_rate_ms s)
     ;; float64 _distance_between_wheels
       (sys::poke _distance_between_wheels (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wheel_radius
       (sys::poke _wheel_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _odom_update_rate_ms
     (setq _odom_update_rate_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _distance_between_wheels
     (setq _distance_between_wheels (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wheel_radius
     (setq _wheel_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass statek_hw::SetOdomParamsResponse
  :super ros::object
  :slots (_success ))

(defmethod statek_hw::SetOdomParamsResponse
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

(defclass statek_hw::SetOdomParams
  :super ros::object
  :slots ())

(setf (get statek_hw::SetOdomParams :md5sum-) "1c8fcad49bde18e0cd305a59dcc09b29")
(setf (get statek_hw::SetOdomParams :datatype-) "statek_hw/SetOdomParams")
(setf (get statek_hw::SetOdomParams :request) statek_hw::SetOdomParamsRequest)
(setf (get statek_hw::SetOdomParams :response) statek_hw::SetOdomParamsResponse)

(defmethod statek_hw::SetOdomParamsRequest
  (:response () (instance statek_hw::SetOdomParamsResponse :init)))

(setf (get statek_hw::SetOdomParamsRequest :md5sum-) "1c8fcad49bde18e0cd305a59dcc09b29")
(setf (get statek_hw::SetOdomParamsRequest :datatype-) "statek_hw/SetOdomParamsRequest")
(setf (get statek_hw::SetOdomParamsRequest :definition-)
      "uint32 odom_update_rate_ms # Set to 0 to ignore.
float64 distance_between_wheels # Set to negative to ignore.
float64 wheel_radius # Set to negative to ignore.
---
bool success # Set to true on success.
")

(setf (get statek_hw::SetOdomParamsResponse :md5sum-) "1c8fcad49bde18e0cd305a59dcc09b29")
(setf (get statek_hw::SetOdomParamsResponse :datatype-) "statek_hw/SetOdomParamsResponse")
(setf (get statek_hw::SetOdomParamsResponse :definition-)
      "uint32 odom_update_rate_ms # Set to 0 to ignore.
float64 distance_between_wheels # Set to negative to ignore.
float64 wheel_radius # Set to negative to ignore.
---
bool success # Set to true on success.
")



(provide :statek_hw/SetOdomParams "1c8fcad49bde18e0cd305a59dcc09b29")


