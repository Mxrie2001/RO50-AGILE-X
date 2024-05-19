; Auto-generated. Do not edit!


(cl:in-package four_wheel_steering_msgs-msg)


;//! \htmlinclude FourWheelSteeringStamped.msg.html

(cl:defclass <FourWheelSteeringStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type four_wheel_steering_msgs-msg:FourWheelSteering
    :initform (cl:make-instance 'four_wheel_steering_msgs-msg:FourWheelSteering)))
)

(cl:defclass FourWheelSteeringStamped (<FourWheelSteeringStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FourWheelSteeringStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FourWheelSteeringStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name four_wheel_steering_msgs-msg:<FourWheelSteeringStamped> is deprecated: use four_wheel_steering_msgs-msg:FourWheelSteeringStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FourWheelSteeringStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:header-val is deprecated.  Use four_wheel_steering_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <FourWheelSteeringStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:data-val is deprecated.  Use four_wheel_steering_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FourWheelSteeringStamped>) ostream)
  "Serializes a message object of type '<FourWheelSteeringStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FourWheelSteeringStamped>) istream)
  "Deserializes a message object of type '<FourWheelSteeringStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FourWheelSteeringStamped>)))
  "Returns string type for a message object of type '<FourWheelSteeringStamped>"
  "four_wheel_steering_msgs/FourWheelSteeringStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FourWheelSteeringStamped)))
  "Returns string type for a message object of type 'FourWheelSteeringStamped"
  "four_wheel_steering_msgs/FourWheelSteeringStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FourWheelSteeringStamped>)))
  "Returns md5sum for a message object of type '<FourWheelSteeringStamped>"
  "9226582df815bc6df9e3206bc05923af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FourWheelSteeringStamped)))
  "Returns md5sum for a message object of type 'FourWheelSteeringStamped"
  "9226582df815bc6df9e3206bc05923af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FourWheelSteeringStamped>)))
  "Returns full string definition for message of type '<FourWheelSteeringStamped>"
  (cl:format cl:nil "## Time stamped drive command or odometry for robots with FourWheelSteering.~%#  $Id$~%~%Header          header~%FourWheelSteering  data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: four_wheel_steering_msgs/FourWheelSteering~%## Driving command or odometry msg for a FourWheelSteering vehicle.~%#  $Id$~%~%# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left~%# and right front wheels are generally at different angles. To simplify,~%# the commanded angle corresponds to the yaw of a virtual wheel located at the~%# center of the front or rear axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates an absolute~%# rate of change either left or right.~%#~%float32 front_steering_angle           # position of the virtual angle (radians)~%float32 rear_steering_angle            # position of the virtual angle (radians)~%float32 front_steering_angle_velocity  # rate of change (radians/s)~%float32 rear_steering_angle_velocity   # rate of change (radians/s)~%~%# Speed, acceleration and jerk (the 1st, 2nd and 3rd~%# derivatives of position). All are measured at the vehicle's~%# center of the rear axle.~%#~%# Speed is the scalar magnitude of the velocity vector.~%# The speed value is the norm of the velocity component in x (longitudinal) ~%# and y (lateral) direction~%# Direction is forward unless the sign is negative, indicating reverse.~%# If the steering angle are equal to +/- pi/2, then the direction is left~%# unless the sign is negative, indicating right.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates an absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates an absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # forward speed (m/s)~%float32 acceleration            # acceleration (m/s^2)~%float32 jerk                    # jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FourWheelSteeringStamped)))
  "Returns full string definition for message of type 'FourWheelSteeringStamped"
  (cl:format cl:nil "## Time stamped drive command or odometry for robots with FourWheelSteering.~%#  $Id$~%~%Header          header~%FourWheelSteering  data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: four_wheel_steering_msgs/FourWheelSteering~%## Driving command or odometry msg for a FourWheelSteering vehicle.~%#  $Id$~%~%# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left~%# and right front wheels are generally at different angles. To simplify,~%# the commanded angle corresponds to the yaw of a virtual wheel located at the~%# center of the front or rear axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates an absolute~%# rate of change either left or right.~%#~%float32 front_steering_angle           # position of the virtual angle (radians)~%float32 rear_steering_angle            # position of the virtual angle (radians)~%float32 front_steering_angle_velocity  # rate of change (radians/s)~%float32 rear_steering_angle_velocity   # rate of change (radians/s)~%~%# Speed, acceleration and jerk (the 1st, 2nd and 3rd~%# derivatives of position). All are measured at the vehicle's~%# center of the rear axle.~%#~%# Speed is the scalar magnitude of the velocity vector.~%# The speed value is the norm of the velocity component in x (longitudinal) ~%# and y (lateral) direction~%# Direction is forward unless the sign is negative, indicating reverse.~%# If the steering angle are equal to +/- pi/2, then the direction is left~%# unless the sign is negative, indicating right.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates an absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates an absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # forward speed (m/s)~%float32 acceleration            # acceleration (m/s^2)~%float32 jerk                    # jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FourWheelSteeringStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FourWheelSteeringStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'FourWheelSteeringStamped
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
