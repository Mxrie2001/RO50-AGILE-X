// Auto-generated. Do not edit!

// (in-package four_wheel_steering_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FourWheelSteering = require('./FourWheelSteering.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FourWheelSteeringStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new FourWheelSteering();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FourWheelSteeringStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = FourWheelSteering.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FourWheelSteeringStamped
    let len;
    let data = new FourWheelSteeringStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = FourWheelSteering.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'four_wheel_steering_msgs/FourWheelSteeringStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9226582df815bc6df9e3206bc05923af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ## Time stamped drive command or odometry for robots with FourWheelSteering.
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FourWheelSteeringStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.data !== undefined) {
      resolved.data = FourWheelSteering.Resolve(msg.data)
    }
    else {
      resolved.data = new FourWheelSteering()
    }

    return resolved;
    }
};

module.exports = FourWheelSteeringStamped;
