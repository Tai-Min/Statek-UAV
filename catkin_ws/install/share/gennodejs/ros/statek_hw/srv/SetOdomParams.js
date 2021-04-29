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

class SetOdomParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.odom_update_rate_ms = null;
      this.distance_between_wheels = null;
      this.wheel_radius = null;
    }
    else {
      if (initObj.hasOwnProperty('odom_update_rate_ms')) {
        this.odom_update_rate_ms = initObj.odom_update_rate_ms
      }
      else {
        this.odom_update_rate_ms = 0;
      }
      if (initObj.hasOwnProperty('distance_between_wheels')) {
        this.distance_between_wheels = initObj.distance_between_wheels
      }
      else {
        this.distance_between_wheels = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_radius')) {
        this.wheel_radius = initObj.wheel_radius
      }
      else {
        this.wheel_radius = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetOdomParamsRequest
    // Serialize message field [odom_update_rate_ms]
    bufferOffset = _serializer.uint32(obj.odom_update_rate_ms, buffer, bufferOffset);
    // Serialize message field [distance_between_wheels]
    bufferOffset = _serializer.float64(obj.distance_between_wheels, buffer, bufferOffset);
    // Serialize message field [wheel_radius]
    bufferOffset = _serializer.float64(obj.wheel_radius, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOdomParamsRequest
    let len;
    let data = new SetOdomParamsRequest(null);
    // Deserialize message field [odom_update_rate_ms]
    data.odom_update_rate_ms = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [distance_between_wheels]
    data.distance_between_wheels = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wheel_radius]
    data.wheel_radius = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/SetOdomParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2c1b0d7cf2d7ec7ca5237bbb9467136';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 odom_update_rate_ms # Set to 0 to ignore.
    float64 distance_between_wheels # Set to negative to ignore.
    float64 wheel_radius # Set to negative to ignore.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetOdomParamsRequest(null);
    if (msg.odom_update_rate_ms !== undefined) {
      resolved.odom_update_rate_ms = msg.odom_update_rate_ms;
    }
    else {
      resolved.odom_update_rate_ms = 0
    }

    if (msg.distance_between_wheels !== undefined) {
      resolved.distance_between_wheels = msg.distance_between_wheels;
    }
    else {
      resolved.distance_between_wheels = 0.0
    }

    if (msg.wheel_radius !== undefined) {
      resolved.wheel_radius = msg.wheel_radius;
    }
    else {
      resolved.wheel_radius = 0.0
    }

    return resolved;
    }
};

class SetOdomParamsResponse {
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
    // Serializes a message object of type SetOdomParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOdomParamsResponse
    let len;
    let data = new SetOdomParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/SetOdomParamsResponse';
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
    const resolved = new SetOdomParamsResponse(null);
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
  Request: SetOdomParamsRequest,
  Response: SetOdomParamsResponse,
  md5sum() { return '1c8fcad49bde18e0cd305a59dcc09b29'; },
  datatype() { return 'statek_hw/SetOdomParams'; }
};
