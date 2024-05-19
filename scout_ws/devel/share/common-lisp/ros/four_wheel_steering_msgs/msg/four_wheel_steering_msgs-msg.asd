
(cl:in-package :asdf)

(defsystem "four_wheel_steering_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FourWheelSteering" :depends-on ("_package_FourWheelSteering"))
    (:file "_package_FourWheelSteering" :depends-on ("_package"))
    (:file "FourWheelSteeringStamped" :depends-on ("_package_FourWheelSteeringStamped"))
    (:file "_package_FourWheelSteeringStamped" :depends-on ("_package"))
  ))