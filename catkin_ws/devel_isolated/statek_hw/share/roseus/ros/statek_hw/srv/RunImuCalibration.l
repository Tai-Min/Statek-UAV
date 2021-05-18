;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::RunImuCalibration)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'RunImuCalibration (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::RUNIMUCALIBRATION")
  (make-package "STATEK_HW::RUNIMUCALIBRATION"))
(unless (find-package "STATEK_HW::RUNIMUCALIBRATIONREQUEST")
  (make-package "STATEK_HW::RUNIMUCALIBRATIONREQUEST"))
(unless (find-package "STATEK_HW::RUNIMUCALIBRATIONRESPONSE")
  (make-package "STATEK_HW::RUNIMUCALIBRATIONRESPONSE"))

(in-package "ROS")





(defclass statek_hw::RunImuCalibrationRequest
  :super ros::object
  :slots ())

(defmethod statek_hw::RunImuCalibrationRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass statek_hw::RunImuCalibrationResponse
  :super ros::object
  :slots (_success _acc_bias _gyro_bias _mag_bias _mag_scale ))

(defmethod statek_hw::RunImuCalibrationResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:acc_bias __acc_bias) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:gyro_bias __gyro_bias) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mag_bias __mag_bias) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mag_scale __mag_scale) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _success __success)
   (setq _acc_bias __acc_bias)
   (setq _gyro_bias __gyro_bias)
   (setq _mag_bias __mag_bias)
   (setq _mag_scale __mag_scale)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
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
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; float64[3] _acc_bias
    (* 8    3)
    ;; float64[3] _gyro_bias
    (* 8    3)
    ;; float64[3] _mag_bias
    (* 8    3)
    ;; float64[3] _mag_scale
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
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
   ;;
   self)
  )

(defclass statek_hw::RunImuCalibration
  :super ros::object
  :slots ())

(setf (get statek_hw::RunImuCalibration :md5sum-) "6cd60c1db4de1bdcd039c0b56b7d1d09")
(setf (get statek_hw::RunImuCalibration :datatype-) "statek_hw/RunImuCalibration")
(setf (get statek_hw::RunImuCalibration :request) statek_hw::RunImuCalibrationRequest)
(setf (get statek_hw::RunImuCalibration :response) statek_hw::RunImuCalibrationResponse)

(defmethod statek_hw::RunImuCalibrationRequest
  (:response () (instance statek_hw::RunImuCalibrationResponse :init)))

(setf (get statek_hw::RunImuCalibrationRequest :md5sum-) "6cd60c1db4de1bdcd039c0b56b7d1d09")
(setf (get statek_hw::RunImuCalibrationRequest :datatype-) "statek_hw/RunImuCalibrationRequest")
(setf (get statek_hw::RunImuCalibrationRequest :definition-)
      "---
bool success # Set to true on success
float64[3] acc_bias # Some value from calibrated MPU9250.
float64[3] gyro_bias # Some value from calibrated MPU9250.
float64[3] mag_bias # Some value from calibrated MPU9250.
float64[3] mag_scale # Some value from calibrated MPU9250.
")

(setf (get statek_hw::RunImuCalibrationResponse :md5sum-) "6cd60c1db4de1bdcd039c0b56b7d1d09")
(setf (get statek_hw::RunImuCalibrationResponse :datatype-) "statek_hw/RunImuCalibrationResponse")
(setf (get statek_hw::RunImuCalibrationResponse :definition-)
      "---
bool success # Set to true on success
float64[3] acc_bias # Some value from calibrated MPU9250.
float64[3] gyro_bias # Some value from calibrated MPU9250.
float64[3] mag_bias # Some value from calibrated MPU9250.
float64[3] mag_scale # Some value from calibrated MPU9250.
")



(provide :statek_hw/RunImuCalibration "6cd60c1db4de1bdcd039c0b56b7d1d09")


