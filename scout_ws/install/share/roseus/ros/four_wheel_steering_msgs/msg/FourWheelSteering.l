;; Auto-generated. Do not edit!


(when (boundp 'four_wheel_steering_msgs::FourWheelSteering)
  (if (not (find-package "FOUR_WHEEL_STEERING_MSGS"))
    (make-package "FOUR_WHEEL_STEERING_MSGS"))
  (shadow 'FourWheelSteering (find-package "FOUR_WHEEL_STEERING_MSGS")))
(unless (find-package "FOUR_WHEEL_STEERING_MSGS::FOURWHEELSTEERING")
  (make-package "FOUR_WHEEL_STEERING_MSGS::FOURWHEELSTEERING"))

(in-package "ROS")
;;//! \htmlinclude FourWheelSteering.msg.html


(defclass four_wheel_steering_msgs::FourWheelSteering
  :super ros::object
  :slots (_front_steering_angle _rear_steering_angle _front_steering_angle_velocity _rear_steering_angle_velocity _speed _acceleration _jerk ))

(defmethod four_wheel_steering_msgs::FourWheelSteering
  (:init
   (&key
    ((:front_steering_angle __front_steering_angle) 0.0)
    ((:rear_steering_angle __rear_steering_angle) 0.0)
    ((:front_steering_angle_velocity __front_steering_angle_velocity) 0.0)
    ((:rear_steering_angle_velocity __rear_steering_angle_velocity) 0.0)
    ((:speed __speed) 0.0)
    ((:acceleration __acceleration) 0.0)
    ((:jerk __jerk) 0.0)
    )
   (send-super :init)
   (setq _front_steering_angle (float __front_steering_angle))
   (setq _rear_steering_angle (float __rear_steering_angle))
   (setq _front_steering_angle_velocity (float __front_steering_angle_velocity))
   (setq _rear_steering_angle_velocity (float __rear_steering_angle_velocity))
   (setq _speed (float __speed))
   (setq _acceleration (float __acceleration))
   (setq _jerk (float __jerk))
   self)
  (:front_steering_angle
   (&optional __front_steering_angle)
   (if __front_steering_angle (setq _front_steering_angle __front_steering_angle)) _front_steering_angle)
  (:rear_steering_angle
   (&optional __rear_steering_angle)
   (if __rear_steering_angle (setq _rear_steering_angle __rear_steering_angle)) _rear_steering_angle)
  (:front_steering_angle_velocity
   (&optional __front_steering_angle_velocity)
   (if __front_steering_angle_velocity (setq _front_steering_angle_velocity __front_steering_angle_velocity)) _front_steering_angle_velocity)
  (:rear_steering_angle_velocity
   (&optional __rear_steering_angle_velocity)
   (if __rear_steering_angle_velocity (setq _rear_steering_angle_velocity __rear_steering_angle_velocity)) _rear_steering_angle_velocity)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:jerk
   (&optional __jerk)
   (if __jerk (setq _jerk __jerk)) _jerk)
  (:serialization-length
   ()
   (+
    ;; float32 _front_steering_angle
    4
    ;; float32 _rear_steering_angle
    4
    ;; float32 _front_steering_angle_velocity
    4
    ;; float32 _rear_steering_angle_velocity
    4
    ;; float32 _speed
    4
    ;; float32 _acceleration
    4
    ;; float32 _jerk
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _front_steering_angle
       (sys::poke _front_steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_steering_angle
       (sys::poke _rear_steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_steering_angle_velocity
       (sys::poke _front_steering_angle_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_steering_angle_velocity
       (sys::poke _rear_steering_angle_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jerk
       (sys::poke _jerk (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _front_steering_angle
     (setq _front_steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_steering_angle
     (setq _rear_steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_steering_angle_velocity
     (setq _front_steering_angle_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_steering_angle_velocity
     (setq _rear_steering_angle_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jerk
     (setq _jerk (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get four_wheel_steering_msgs::FourWheelSteering :md5sum-) "04dd0f55e1f168668af1e2587a7cdd2a")
(setf (get four_wheel_steering_msgs::FourWheelSteering :datatype-) "four_wheel_steering_msgs/FourWheelSteering")
(setf (get four_wheel_steering_msgs::FourWheelSteering :definition-)
      "## Driving command or odometry msg for a FourWheelSteering vehicle.
#  $Id$

# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left
# and right front wheels are generally at different angles. To simplify,
# the commanded angle corresponds to the yaw of a virtual wheel located at the
# center of the front or rear axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates an absolute
# rate of change either left or right.
#
float32 front_steering_angle           # position of the virtual angle (radians)
float32 rear_steering_angle            # position of the virtual angle (radians)
float32 front_steering_angle_velocity  # rate of change (radians/s)
float32 rear_steering_angle_velocity   # rate of change (radians/s)

# Speed, acceleration and jerk (the 1st, 2nd and 3rd
# derivatives of position). All are measured at the vehicle's
# center of the rear axle.
#
# Speed is the scalar magnitude of the velocity vector.
# The speed value is the norm of the velocity component in x (longitudinal) 
# and y (lateral) direction
# Direction is forward unless the sign is negative, indicating reverse.
# If the steering angle are equal to +/- pi/2, then the direction is left
# unless the sign is negative, indicating right.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates an absolute
# magnitude; that includes deceleration.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk indicates an absolute rate of acceleration change in
# either direction (increasing or decreasing).
#
float32 speed                   # forward speed (m/s)
float32 acceleration            # acceleration (m/s^2)
float32 jerk                    # jerk (m/s^3)

")



(provide :four_wheel_steering_msgs/FourWheelSteering "04dd0f55e1f168668af1e2587a7cdd2a")


