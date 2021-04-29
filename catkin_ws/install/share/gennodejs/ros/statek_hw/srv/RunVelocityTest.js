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

class RunVelocityTestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_ms = null;
    }
    else {
      if (initObj.hasOwnProperty('time_ms')) {
        this.time_ms = initObj.time_ms
      }
      else {
        this.time_ms = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunVelocityTestRequest
    // Serialize message field [time_ms]
    bufferOffset = _serializer.uint32(obj.time_ms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunVelocityTestRequest
    let len;
    let data = new RunVelocityTestRequest(null);
    // Deserialize message field [time_ms]
    data.time_ms = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/RunVelocityTestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6301565e50e0b100332dde1661ab3ebc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 time_ms # Test time in milliseconds.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunVelocityTestRequest(null);
    if (msg.time_ms !== undefined) {
      resolved.time_ms = msg.time_ms;
    }
    else {
      resolved.time_ms = 0
    }

    return resolved;
    }
};

class RunVelocityTestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunVelocityTestResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunVelocityTestResponse
    let len;
    let data = new RunVelocityTestResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/RunVelocityTestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '738dd6bd6e6687eab1176abf1ea50abc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success # Set to true on success
    float64 velocity # Found velocity in rad/s.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunVelocityTestResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: RunVelocityTestRequest,
  Response: RunVelocityTestResponse,
  md5sum() { return '0da28cc5facb55eb49569bd980fe316c'; },
  datatype() { return 'statek_hw/RunVelocityTest'; }
};
