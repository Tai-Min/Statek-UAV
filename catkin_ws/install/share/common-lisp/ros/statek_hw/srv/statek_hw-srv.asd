
(cl:in-package :asdf)

(defsystem "statek_hw-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RunImuCalibration" :depends-on ("_package_RunImuCalibration"))
    (:file "_package_RunImuCalibration" :depends-on ("_package"))
    (:file "RunModelIdentification" :depends-on ("_package_RunModelIdentification"))
    (:file "_package_RunModelIdentification" :depends-on ("_package"))
    (:file "RunVelocityTest" :depends-on ("_package_RunVelocityTest"))
    (:file "_package_RunVelocityTest" :depends-on ("_package"))
    (:file "SetImuParams" :depends-on ("_package_SetImuParams"))
    (:file "_package_SetImuParams" :depends-on ("_package"))
    (:file "SetMotorParams" :depends-on ("_package_SetMotorParams"))
    (:file "_package_SetMotorParams" :depends-on ("_package"))
    (:file "SetOdomParams" :depends-on ("_package_SetOdomParams"))
    (:file "_package_SetOdomParams" :depends-on ("_package"))
  ))