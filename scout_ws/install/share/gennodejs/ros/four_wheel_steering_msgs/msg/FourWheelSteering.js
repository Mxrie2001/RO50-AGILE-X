// Auto-generated. Do not edit!

// (in-package four_wheel_steering_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FourWheelSteering {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.front_steering_angle = null;
      this.rear_steering_angle = null;
      this.front_steering_angle_velocity = null;
      this.rear_steering_angle_velocity = null;
      this.speed = null;
      this.acceleration = null;
      this.jerk = null;
    }
    else {
      if (initObj.hasOwnProperty('front_steering_angle')) {
        this.front_steering_angle = initObj.front_steering_angle
      }
      else {
        this.front_steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('rear_steering_angle')) {
        this.rear_steering_angle = initObj.rear_steering_angle
      }
      else {
        this.rear_steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('front_steering_angle_velocity')) {
        this.front_steering_angle_velocity = initObj.front_steering_angle_velocity
      }
      else {
        this.front_steering_angle_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('rear_steering_angle_velocity')) {
        this.rear_steering_angle_velocity = initObj.rear_steering_angle_velocity
      }
      else {
        this.rear_steering_angle_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = 0.0;
      }
      if (initObj.hasOwnProperty('jerk')) {
        this.jerk = initObj.jerk
      }
      else {
        this.jerk = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FourWheelSteering
    // Serialize message field [front_steering_angle]
    bufferOffset = _serializer.float32(obj.front_steering_angle, buffer, bufferOffset);
    // Serialize message field [rear_steering_angle]
    bufferOffset = _serializer.float32(obj.rear_steering_angle, buffer, bufferOffset);
    // Serialize message field [front_steering_angle_velocity]
    bufferOffset = _serializer.float32(obj.front_steering_angle_velocity, buffer, bufferOffset);
    // Serialize message field [rear_steering_angle_velocity]
    bufferOffset = _serializer.float32(obj.rear_steering_angle_velocity, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = _serializer.float32(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [jerk]
    bufferOffset = _serializer.float32(obj.jerk, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FourWheelSteering
    let len;
    let data = new FourWheelSteering(null);
    // Deserialize message field [front_steering_angle]
    data.front_steering_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rear_steering_angle]
    data.rear_steering_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [front_steering_angle_velocity]
    data.front_steering_angle_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rear_steering_angle_velocity]
    data.rear_steering_angle_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jerk]
    data.jerk = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'four_wheel_steering_msgs/FourWheelSteering';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04dd0f55e1f168668af1e2587a7cdd2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FourWheelSteering(null);
    if (msg.front_steering_angle !== undefined) {
      resolved.front_steering_angle = msg.front_steering_angle;
    }
    else {
      resolved.front_steering_angle = 0.0
    }

    if (msg.rear_steering_angle !== undefined) {
      resolved.rear_steering_angle = msg.rear_steering_angle;
    }
    else {
      resolved.rear_steering_angle = 0.0
    }

    if (msg.front_steering_angle_velocity !== undefined) {
      resolved.front_steering_angle_velocity = msg.front_steering_angle_velocity;
    }
    else {
      resolved.front_steering_angle_velocity = 0.0
    }

    if (msg.rear_steering_angle_velocity !== undefined) {
      resolved.rear_steering_angle_velocity = msg.rear_steering_angle_velocity;
    }
    else {
      resolved.rear_steering_angle_velocity = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = 0.0
    }

    if (msg.jerk !== undefined) {
      resolved.jerk = msg.jerk;
    }
    else {
      resolved.jerk = 0.0
    }

    return resolved;
    }
};

module.exports = FourWheelSteering;
