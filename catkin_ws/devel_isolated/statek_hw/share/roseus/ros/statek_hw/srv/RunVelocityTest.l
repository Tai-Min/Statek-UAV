;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::RunVelocityTest)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'RunVelocityTest (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::RUNVELOCITYTEST")
  (make-package "STATEK_HW::RUNVELOCITYTEST"))
(unless (find-package "STATEK_HW::RUNVELOCITYTESTREQUEST")
  (make-package "STATEK_HW::RUNVELOCITYTESTREQUEST"))
(unless (find-package "STATEK_HW::RUNVELOCITYTESTRESPONSE")
  (make-package "STATEK_HW::RUNVELOCITYTESTRESPONSE"))

(in-package "ROS")





(defclass statek_hw::RunVelocityTestRequest
  :super ros::object
  :slots (_time_ms ))

(defmethod statek_hw::RunVelocityTestRequest
  (:init
   (&key
    ((:time_ms __time_ms) 0)
    )
   (send-super :init)
   (setq _time_ms (round __time_ms))
   self)
  (:time_ms
   (&optional __time_ms)
   (if __time_ms (setq _time_ms __time_ms)) _time_ms)
  (:serialization-length
   ()
   (+
    ;; uint32 _time_ms
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _time_ms
       (write-long _time_ms s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _time_ms
     (setq _time_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass statek_hw::RunVelocityTestResponse
  :super ros::object
  :slots (_success _velocity ))

(defmethod statek_hw::RunVelocityTestResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:velocity __velocity) 0.0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _velocity (float __velocity))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; float64 _velocity
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass statek_hw::RunVelocityTest
  :super ros::object
  :slots ())

(setf (get statek_hw::RunVelocityTest :md5sum-) "0da28cc5facb55eb49569bd980fe316c")
(setf (get statek_hw::RunVelocityTest :datatype-) "statek_hw/RunVelocityTest")
(setf (get statek_hw::RunVelocityTest :request) statek_hw::RunVelocityTestRequest)
(setf (get statek_hw::RunVelocityTest :response) statek_hw::RunVelocityTestResponse)

(defmethod statek_hw::RunVelocityTestRequest
  (:response () (instance statek_hw::RunVelocityTestResponse :init)))

(setf (get statek_hw::RunVelocityTestRequest :md5sum-) "0da28cc5facb55eb49569bd980fe316c")
(setf (get statek_hw::RunVelocityTestRequest :datatype-) "statek_hw/RunVelocityTestRequest")
(setf (get statek_hw::RunVelocityTestRequest :definition-)
      "uint32 time_ms # Test time in milliseconds.
---
bool success # Set to true on success
float64 velocity # Found velocity in rad/s.
")

(setf (get statek_hw::RunVelocityTestResponse :md5sum-) "0da28cc5facb55eb49569bd980fe316c")
(setf (get statek_hw::RunVelocityTestResponse :datatype-) "statek_hw/RunVelocityTestResponse")
(setf (get statek_hw::RunVelocityTestResponse :definition-)
      "uint32 time_ms # Test time in milliseconds.
---
bool success # Set to true on success
float64 velocity # Found velocity in rad/s.
")



(provide :statek_hw/RunVelocityTest "0da28cc5facb55eb49569bd980fe316c")


