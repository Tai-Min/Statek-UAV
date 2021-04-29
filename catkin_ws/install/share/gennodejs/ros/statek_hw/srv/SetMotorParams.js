// Auto-generated. Do not edit!

// (in-package statek_hw.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetMotorParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loop_update_rate_ms = null;
      this.wheel_max_angular_velocity = null;
      this.smoothing_factor = null;
      this.kp = null;
      this.ki = null;
      this.kd = null;
    }
    else {
      if (initObj.hasOwnProperty('loop_update_rate_ms')) {
        this.loop_update_rate_ms = initObj.loop_update_rate_ms
      }
      else {
        this.loop_update_rate_ms = 0;
      }
      if (initObj.hasOwnProperty('wheel_max_angular_velocity')) {
        this.wheel_max_angular_velocity = initObj.wheel_max_angular_velocity
      }
      else {
        this.wheel_max_angular_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('smoothing_factor')) {
        this.smoothing_factor = initObj.smoothing_factor
      }
      else {
        this.smoothing_factor = 0.0;
      }
      if (initObj.hasOwnProperty('kp')) {
        this.kp = initObj.kp
      }
      else {
        this.kp = 0.0;
      }
      if (initObj.hasOwnProperty('ki')) {
        this.ki = initObj.ki
      }
      else {
        this.ki = 0.0;
      }
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMotorParamsRequest
    // Serialize message field [loop_update_rate_ms]
    bufferOffset = _serializer.uint32(obj.loop_update_rate_ms, buffer, bufferOffset);
    // Serialize message field [wheel_max_angular_velocity]
    bufferOffset = _serializer.float64(obj.wheel_max_angular_velocity, buffer, bufferOffset);
    // Serialize message field [smoothing_factor]
    bufferOffset = _serializer.float64(obj.smoothing_factor, buffer, bufferOffset);
    // Serialize message field [kp]
    bufferOffset = _serializer.float64(obj.kp, buffer, bufferOffset);
    // Serialize message field [ki]
    bufferOffset = _serializer.float64(obj.ki, buffer, bufferOffset);
    // Serialize message field [kd]
    bufferOffset = _serializer.float64(obj.kd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMotorParamsRequest
    let len;
    let data = new SetMotorParamsRequest(null);
    // Deserialize message field [loop_update_rate_ms]
    data.loop_update_rate_ms = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [wheel_max_angular_velocity]
    data.wheel_max_angular_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [smoothing_factor]
    data.smoothing_factor = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [kp]
    data.kp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ki]
    data.ki = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [kd]
    data.kd = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/SetMotorParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8b46f7af46e051c167d322324998b79';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 loop_update_rate_ms # In ms. Set to 0 to ignore.
    float64 wheel_max_angular_velocity # In rad/s. Set to negative to ignore.
    float64 smoothing_factor # Between 0 and 1. Set to negative to ignore.
    float64 kp # Set to negative to ignore.
    float64 ki # Set to negative to ignore.
    float64 kd # Set to negative to ignore.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMotorParamsRequest(null);
    if (msg.loop_update_rate_ms !== undefined) {
      resolved.loop_update_rate_ms = msg.loop_update_rate_ms;
    }
    else {
      resolved.loop_update_rate_ms = 0
    }

    if (msg.wheel_max_angular_velocity !== undefined) {
      resolved.wheel_max_angular_velocity = msg.wheel_max_angular_velocity;
    }
    else {
      resolved.wheel_max_angular_velocity = 0.0
    }

    if (msg.smoothing_factor !== undefined) {
      resolved.smoothing_factor = msg.smoothing_factor;
    }
    else {
      resolved.smoothing_factor = 0.0
    }

    if (msg.kp !== undefined) {
      resolved.kp = msg.kp;
    }
    else {
      resolved.kp = 0.0
    }

    if (msg.ki !== undefined) {
      resolved.ki = msg.ki;
    }
    else {
      resolved.ki = 0.0
    }

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = 0.0
    }

    return resolved;
    }
};

class SetMotorParamsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMotorParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMotorParamsResponse
    let len;
    let data = new SetMotorParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/SetMotorParamsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success # Set to true on success.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMotorParamsResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetMotorParamsRequest,
  Response: SetMotorParamsResponse,
  md5sum() { return 'f3ac856c8e72619c8db00906cbcd4cd3'; },
  datatype() { return 'statek_hw/SetMotorParams'; }
};
