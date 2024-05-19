;; Auto-generated. Do not edit!


(when (boundp 'four_wheel_steering_msgs::FourWheelSteeringStamped)
  (if (not (find-package "FOUR_WHEEL_STEERING_MSGS"))
    (make-package "FOUR_WHEEL_STEERING_MSGS"))
  (shadow 'FourWheelSteeringStamped (find-package "FOUR_WHEEL_STEERING_MSGS")))
(unless (find-package "FOUR_WHEEL_STEERING_MSGS::FOURWHEELSTEERINGSTAMPED")
  (make-package "FOUR_WHEEL_STEERING_MSGS::FOURWHEELSTEERINGSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude FourWheelSteeringStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass four_wheel_steering_msgs::FourWheelSteeringStamped
  :super ros::object
  :slots (_header _data ))

(defmethod four_wheel_steering_msgs::FourWheelSteeringStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (instance four_wheel_steering_msgs::FourWheelSteering :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; four_wheel_steering_msgs/FourWheelSteering _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; four_wheel_steering_msgs/FourWheelSteering _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; four_wheel_steering_msgs/FourWheelSteering _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(setf (get four_wheel_steering_msgs::FourWheelSteeringStamped :md5sum-) "9226582df815bc6df9e3206bc05923af")
(setf (get four_wheel_steering_msgs::FourWheelSteeringStamped :datatype-) "four_wheel_steering_msgs/FourWheelSteeringStamped")
(setf (get four_wheel_steering_msgs::FourWheelSteeringStamped :definition-)
      "## Time stamped drive command or odometry for robots with FourWheelSteering.
#  $Id$

Header          header
FourWheelSteering  data

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: four_wheel_steering_msgs/FourWheelSteering
## Driving command or odometry msg for a FourWheelSteering vehicle.
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



(provide :four_wheel_steering_msgs/FourWheelSteeringStamped "9226582df815bc6df9e3206bc05923af")


