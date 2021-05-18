;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::SetMotorParams)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'SetMotorParams (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::SETMOTORPARAMS")
  (make-package "STATEK_HW::SETMOTORPARAMS"))
(unless (find-package "STATEK_HW::SETMOTORPARAMSREQUEST")
  (make-package "STATEK_HW::SETMOTORPARAMSREQUEST"))
(unless (find-package "STATEK_HW::SETMOTORPARAMSRESPONSE")
  (make-package "STATEK_HW::SETMOTORPARAMSRESPONSE"))

(in-package "ROS")





(defclass statek_hw::SetMotorParamsRequest
  :super ros::object
  :slots (_loop_update_rate_ms _wheel_max_angular_velocity _smoothing_factor _kp _ki _kd ))

(defmethod statek_hw::SetMotorParamsRequest
  (:init
   (&key
    ((:loop_update_rate_ms __loop_update_rate_ms) 0)
    ((:wheel_max_angular_velocity __wheel_max_angular_velocity) 0.0)
    ((:smoothing_factor __smoothing_factor) 0.0)
    ((:kp __kp) 0.0)
    ((:ki __ki) 0.0)
    ((:kd __kd) 0.0)
    )
   (send-super :init)
   (setq _loop_update_rate_ms (round __loop_update_rate_ms))
   (setq _wheel_max_angular_velocity (float __wheel_max_angular_velocity))
   (setq _smoothing_factor (float __smoothing_factor))
   (setq _kp (float __kp))
   (setq _ki (float __ki))
   (setq _kd (float __kd))
   self)
  (:loop_update_rate_ms
   (&optional __loop_update_rate_ms)
   (if __loop_update_rate_ms (setq _loop_update_rate_ms __loop_update_rate_ms)) _loop_update_rate_ms)
  (:wheel_max_angular_velocity
   (&optional __wheel_max_angular_velocity)
   (if __wheel_max_angular_velocity (setq _wheel_max_angular_velocity __wheel_max_angular_velocity)) _wheel_max_angular_velocity)
  (:smoothing_factor
   (&optional __smoothing_factor)
   (if __smoothing_factor (setq _smoothing_factor __smoothing_factor)) _smoothing_factor)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:ki
   (&optional __ki)
   (if __ki (setq _ki __ki)) _ki)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:serialization-length
   ()
   (+
    ;; uint32 _loop_update_rate_ms
    4
    ;; float64 _wheel_max_angular_velocity
    8
    ;; float64 _smoothing_factor
    8
    ;; float64 _kp
    8
    ;; float64 _ki
    8
    ;; float64 _kd
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _loop_update_rate_ms
       (write-long _loop_update_rate_ms s)
     ;; float64 _wheel_max_angular_velocity
       (sys::poke _wheel_max_angular_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _smoothing_factor
       (sys::poke _smoothing_factor (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kp
       (sys::poke _kp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ki
       (sys::poke _ki (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kd
       (sys::poke _kd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _loop_update_rate_ms
     (setq _loop_update_rate_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _wheel_max_angular_velocity
     (setq _wheel_max_angular_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _smoothing_factor
     (setq _smoothing_factor (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kp
     (setq _kp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ki
     (setq _ki (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kd
     (setq _kd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass statek_hw::SetMotorParamsResponse
  :super ros::object
  :slots (_success ))

(defmethod statek_hw::SetMotorParamsResponse
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

(defclass statek_hw::SetMotorParams
  :super ros::object
  :slots ())

(setf (get statek_hw::SetMotorParams :md5sum-) "f3ac856c8e72619c8db00906cbcd4cd3")
(setf (get statek_hw::SetMotorParams :datatype-) "statek_hw/SetMotorParams")
(setf (get statek_hw::SetMotorParams :request) statek_hw::SetMotorParamsRequest)
(setf (get statek_hw::SetMotorParams :response) statek_hw::SetMotorParamsResponse)

(defmethod statek_hw::SetMotorParamsRequest
  (:response () (instance statek_hw::SetMotorParamsResponse :init)))

(setf (get statek_hw::SetMotorParamsRequest :md5sum-) "f3ac856c8e72619c8db00906cbcd4cd3")
(setf (get statek_hw::SetMotorParamsRequest :datatype-) "statek_hw/SetMotorParamsRequest")
(setf (get statek_hw::SetMotorParamsRequest :definition-)
      "uint32 loop_update_rate_ms # In ms. Set to 0 to ignore.
float64 wheel_max_angular_velocity # In rad/s. Set to negative to ignore.
float64 smoothing_factor # Between 0 and 1. Set to negative to ignore.
float64 kp # Set to negative to ignore.
float64 ki # Set to negative to ignore.
float64 kd # Set to negative to ignore.
---
bool success # Set to true on success.
")

(setf (get statek_hw::SetMotorParamsResponse :md5sum-) "f3ac856c8e72619c8db00906cbcd4cd3")
(setf (get statek_hw::SetMotorParamsResponse :datatype-) "statek_hw/SetMotorParamsResponse")
(setf (get statek_hw::SetMotorParamsResponse :definition-)
      "uint32 loop_update_rate_ms # In ms. Set to 0 to ignore.
float64 wheel_max_angular_velocity # In rad/s. Set to negative to ignore.
float64 smoothing_factor # Between 0 and 1. Set to negative to ignore.
float64 kp # Set to negative to ignore.
float64 ki # Set to negative to ignore.
float64 kd # Set to negative to ignore.
---
bool success # Set to true on success.
")



(provide :statek_hw/SetMotorParams "f3ac856c8e72619c8db00906cbcd4cd3")


