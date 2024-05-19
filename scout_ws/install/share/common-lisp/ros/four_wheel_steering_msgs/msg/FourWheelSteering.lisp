; Auto-generated. Do not edit!


(cl:in-package four_wheel_steering_msgs-msg)


;//! \htmlinclude FourWheelSteering.msg.html

(cl:defclass <FourWheelSteering> (roslisp-msg-protocol:ros-message)
  ((front_steering_angle
    :reader front_steering_angle
    :initarg :front_steering_angle
    :type cl:float
    :initform 0.0)
   (rear_steering_angle
    :reader rear_steering_angle
    :initarg :rear_steering_angle
    :type cl:float
    :initform 0.0)
   (front_steering_angle_velocity
    :reader front_steering_angle_velocity
    :initarg :front_steering_angle_velocity
    :type cl:float
    :initform 0.0)
   (rear_steering_angle_velocity
    :reader rear_steering_angle_velocity
    :initarg :rear_steering_angle_velocity
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (jerk
    :reader jerk
    :initarg :jerk
    :type cl:float
    :initform 0.0))
)

(cl:defclass FourWheelSteering (<FourWheelSteering>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FourWheelSteering>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FourWheelSteering)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name four_wheel_steering_msgs-msg:<FourWheelSteering> is deprecated: use four_wheel_steering_msgs-msg:FourWheelSteering instead.")))

(cl:ensure-generic-function 'front_steering_angle-val :lambda-list '(m))
(cl:defmethod front_steering_angle-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:front_steering_angle-val is deprecated.  Use four_wheel_steering_msgs-msg:front_steering_angle instead.")
  (front_steering_angle m))

(cl:ensure-generic-function 'rear_steering_angle-val :lambda-list '(m))
(cl:defmethod rear_steering_angle-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:rear_steering_angle-val is deprecated.  Use four_wheel_steering_msgs-msg:rear_steering_angle instead.")
  (rear_steering_angle m))

(cl:ensure-generic-function 'front_steering_angle_velocity-val :lambda-list '(m))
(cl:defmethod front_steering_angle_velocity-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:front_steering_angle_velocity-val is deprecated.  Use four_wheel_steering_msgs-msg:front_steering_angle_velocity instead.")
  (front_steering_angle_velocity m))

(cl:ensure-generic-function 'rear_steering_angle_velocity-val :lambda-list '(m))
(cl:defmethod rear_steering_angle_velocity-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:rear_steering_angle_velocity-val is deprecated.  Use four_wheel_steering_msgs-msg:rear_steering_angle_velocity instead.")
  (rear_steering_angle_velocity m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:speed-val is deprecated.  Use four_wheel_steering_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:acceleration-val is deprecated.  Use four_wheel_steering_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'jerk-val :lambda-list '(m))
(cl:defmethod jerk-val ((m <FourWheelSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader four_wheel_steering_msgs-msg:jerk-val is deprecated.  Use four_wheel_steering_msgs-msg:jerk instead.")
  (jerk m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FourWheelSteering>) ostream)
  "Serializes a message object of type '<FourWheelSteering>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_steering_angle_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_steering_angle_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jerk))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FourWheelSteering>) istream)
  "Deserializes a message object of type '<FourWheelSteering>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_steering_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_steering_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_steering_angle_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_steering_angle_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jerk) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FourWheelSteering>)))
  "Returns string type for a message object of type '<FourWheelSteering>"
  "four_wheel_steering_msgs/FourWheelSteering")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FourWheelSteering)))
  "Returns string type for a message object of type 'FourWheelSteering"
  "four_wheel_steering_msgs/FourWheelSteering")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FourWheelSteering>)))
  "Returns md5sum for a message object of type '<FourWheelSteering>"
  "04dd0f55e1f168668af1e2587a7cdd2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FourWheelSteering)))
  "Returns md5sum for a message object of type 'FourWheelSteering"
  "04dd0f55e1f168668af1e2587a7cdd2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FourWheelSteering>)))
  "Returns full string definition for message of type '<FourWheelSteering>"
  (cl:format cl:nil "## Driving command or odometry msg for a FourWheelSteering vehicle.~%#  $Id$~%~%# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left~%# and right front wheels are generally at different angles. To simplify,~%# the commanded angle corresponds to the yaw of a virtual wheel located at the~%# center of the front or rear axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates an absolute~%# rate of change either left or right.~%#~%float32 front_steering_angle           # position of the virtual angle (radians)~%float32 rear_steering_angle            # position of the virtual angle (radians)~%float32 front_steering_angle_velocity  # rate of change (radians/s)~%float32 rear_steering_angle_velocity   # rate of change (radians/s)~%~%# Speed, acceleration and jerk (the 1st, 2nd and 3rd~%# derivatives of position). All are measured at the vehicle's~%# center of the rear axle.~%#~%# Speed is the scalar magnitude of the velocity vector.~%# The speed value is the norm of the velocity component in x (longitudinal) ~%# and y (lateral) direction~%# Direction is forward unless the sign is negative, indicating reverse.~%# If the steering angle are equal to +/- pi/2, then the direction is left~%# unless the sign is negative, indicating right.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates an absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates an absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # forward speed (m/s)~%float32 acceleration            # acceleration (m/s^2)~%float32 jerk                    # jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FourWheelSteering)))
  "Returns full string definition for message of type 'FourWheelSteering"
  (cl:format cl:nil "## Driving command or odometry msg for a FourWheelSteering vehicle.~%#  $Id$~%~%# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left~%# and right front wheels are generally at different angles. To simplify,~%# the commanded angle corresponds to the yaw of a virtual wheel located at the~%# center of the front or rear axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates an absolute~%# rate of change either left or right.~%#~%float32 front_steering_angle           # position of the virtual angle (radians)~%float32 rear_steering_angle            # position of the virtual angle (radians)~%float32 front_steering_angle_velocity  # rate of change (radians/s)~%float32 rear_steering_angle_velocity   # rate of change (radians/s)~%~%# Speed, acceleration and jerk (the 1st, 2nd and 3rd~%# derivatives of position). All are measured at the vehicle's~%# center of the rear axle.~%#~%# Speed is the scalar magnitude of the velocity vector.~%# The speed value is the norm of the velocity component in x (longitudinal) ~%# and y (lateral) direction~%# Direction is forward unless the sign is negative, indicating reverse.~%# If the steering angle are equal to +/- pi/2, then the direction is left~%# unless the sign is negative, indicating right.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates an absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates an absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # forward speed (m/s)~%float32 acceleration            # acceleration (m/s^2)~%float32 jerk                    # jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FourWheelSteering>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FourWheelSteering>))
  "Converts a ROS message object to a list"
  (cl:list 'FourWheelSteering
    (cl:cons ':front_steering_angle (front_steering_angle msg))
    (cl:cons ':rear_steering_angle (rear_steering_angle msg))
    (cl:cons ':front_steering_angle_velocity (front_steering_angle_velocity msg))
    (cl:cons ':rear_steering_angle_velocity (rear_steering_angle_velocity msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':jerk (jerk msg))
))
