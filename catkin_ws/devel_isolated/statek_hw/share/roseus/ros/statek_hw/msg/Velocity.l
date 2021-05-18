;; Auto-generated. Do not edit!


(when (boundp 'statek_hw::Velocity)
  (if (not (find-package "STATEK_HW"))
    (make-package "STATEK_HW"))
  (shadow 'Velocity (find-package "STATEK_HW")))
(unless (find-package "STATEK_HW::VELOCITY")
  (make-package "STATEK_HW::VELOCITY"))

(in-package "ROS")
;;//! \htmlinclude Velocity.msg.html


(defclass statek_hw::Velocity
  :super ros::object
  :slots (_left _right ))

(defmethod statek_hw::Velocity
  (:init
   (&key
    ((:left __left) 0.0)
    ((:right __right) 0.0)
    )
   (send-super :init)
   (setq _left (float __left))
   (setq _right (float __right))
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; float64 _left
    8
    ;; float64 _right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _left
       (sys::poke _left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right
       (sys::poke _right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _left
     (setq _left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right
     (setq _right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get statek_hw::Velocity :md5sum-) "50c2436c38cded221d061b57126c4e40")
(setf (get statek_hw::Velocity :datatype-) "statek_hw/Velocity")
(setf (get statek_hw::Velocity :definition-)
      "float64 left # Left wheel angular velocity in rad/s.
float64 right # Right wheel angular velocity in rad/s.
")



(provide :statek_hw/Velocity "50c2436c38cded221d061b57126c4e40")


