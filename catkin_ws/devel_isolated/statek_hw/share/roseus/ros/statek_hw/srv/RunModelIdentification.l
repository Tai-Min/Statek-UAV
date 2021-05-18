;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::RunModelIdentification)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'RunModelIdentification (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::RUNMODELIDENTIFICATION")
  (make-package "STATEK_HW::RUNMODELIDENTIFICATION"))
(unless (find-package "STATEK_HW::RUNMODELIDENTIFICATIONREQUEST")
  (make-package "STATEK_HW::RUNMODELIDENTIFICATIONREQUEST"))
(unless (find-package "STATEK_HW::RUNMODELIDENTIFICATIONRESPONSE")
  (make-package "STATEK_HW::RUNMODELIDENTIFICATIONRESPONSE"))

(in-package "ROS")





(defclass statek_hw::RunModelIdentificationRequest
  :super ros::object
  :slots (_identification_time_ms ))

(defmethod statek_hw::RunModelIdentificationRequest
  (:init
   (&key
    ((:identification_time_ms __identification_time_ms) 0)
    )
   (send-super :init)
   (setq _identification_time_ms (round __identification_time_ms))
   self)
  (:identification_time_ms
   (&optional __identification_time_ms)
   (if __identification_time_ms (setq _identification_time_ms __identification_time_ms)) _identification_time_ms)
  (:serialization-length
   ()
   (+
    ;; uint32 _identification_time_ms
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _identification_time_ms
       (write-long _identification_time_ms s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identification_time_ms
     (setq _identification_time_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass statek_hw::RunModelIdentificationResponse
  :super ros::object
  :slots (_success _sampling_time _samples ))

(defmethod statek_hw::RunModelIdentificationResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:sampling_time __sampling_time) 0.0)
    ((:samples __samples) (make-array 100 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _success __success)
   (setq _sampling_time (float __sampling_time))
   (setq _samples __samples)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:sampling_time
   (&optional __sampling_time)
   (if __sampling_time (setq _sampling_time __sampling_time)) _sampling_time)
  (:samples
   (&optional __samples)
   (if __samples (setq _samples __samples)) _samples)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; float64 _sampling_time
    8
    ;; float64[100] _samples
    (* 8    100)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64 _sampling_time
       (sys::poke _sampling_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[100] _samples
     (dotimes (i 100)
       (sys::poke (elt _samples i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _sampling_time
     (setq _sampling_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[100] _samples
   (dotimes (i (length _samples))
     (setf (elt _samples i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass statek_hw::RunModelIdentification
  :super ros::object
  :slots ())

(setf (get statek_hw::RunModelIdentification :md5sum-) "d0062710edaa1716482716b90a44c970")
(setf (get statek_hw::RunModelIdentification :datatype-) "statek_hw/RunModelIdentification")
(setf (get statek_hw::RunModelIdentification :request) statek_hw::RunModelIdentificationRequest)
(setf (get statek_hw::RunModelIdentification :response) statek_hw::RunModelIdentificationResponse)

(defmethod statek_hw::RunModelIdentificationRequest
  (:response () (instance statek_hw::RunModelIdentificationResponse :init)))

(setf (get statek_hw::RunModelIdentificationRequest :md5sum-) "d0062710edaa1716482716b90a44c970")
(setf (get statek_hw::RunModelIdentificationRequest :datatype-) "statek_hw/RunModelIdentificationRequest")
(setf (get statek_hw::RunModelIdentificationRequest :definition-)
      "uint32 identification_time_ms
---
bool success
float64 sampling_time
float64[100] samples
")

(setf (get statek_hw::RunModelIdentificationResponse :md5sum-) "d0062710edaa1716482716b90a44c970")
(setf (get statek_hw::RunModelIdentificationResponse :datatype-) "statek_hw/RunModelIdentificationResponse")
(setf (get statek_hw::RunModelIdentificationResponse :definition-)
      "uint32 identification_time_ms
---
bool success
float64 sampling_time
float64[100] samples
")



(provide :statek_hw/RunModelIdentification "d0062710edaa1716482716b90a44c970")


