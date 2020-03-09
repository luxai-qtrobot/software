
(cl:in-package :asdf)

(defsystem "qt_motors_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorState" :depends-on ("_package_MotorState"))
    (:file "_package_MotorState" :depends-on ("_package"))
  ))