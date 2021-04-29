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

class RunModelIdentificationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.identification_time_ms = null;
    }
    else {
      if (initObj.hasOwnProperty('identification_time_ms')) {
        this.identification_time_ms = initObj.identification_time_ms
      }
      else {
        this.identification_time_ms = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunModelIdentificationRequest
    // Serialize message field [identification_time_ms]
    bufferOffset = _serializer.uint32(obj.identification_time_ms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunModelIdentificationRequest
    let len;
    let data = new RunModelIdentificationRequest(null);
    // Deserialize message field [identification_time_ms]
    data.identification_time_ms = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/RunModelIdentificationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a1d96b6498339e94d9e98e6c92d5137';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 identification_time_ms
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunModelIdentificationRequest(null);
    if (msg.identification_time_ms !== undefined) {
      resolved.identification_time_ms = msg.identification_time_ms;
    }
    else {
      resolved.identification_time_ms = 0
    }

    return resolved;
    }
};

class RunModelIdentificationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.sampling_time = null;
      this.samples = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('sampling_time')) {
        this.sampling_time = initObj.sampling_time
      }
      else {
        this.sampling_time = 0.0;
      }
      if (initObj.hasOwnProperty('samples')) {
        this.samples = initObj.samples
      }
      else {
        this.samples = new Array(100).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunModelIdentificationResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [sampling_time]
    bufferOffset = _serializer.float64(obj.sampling_time, buffer, bufferOffset);
    // Check that the constant length array field [samples] has the right length
    if (obj.samples.length !== 100) {
      throw new Error('Unable to serialize array field samples - length must be 100')
    }
    // Serialize message field [samples]
    bufferOffset = _arraySerializer.float64(obj.samples, buffer, bufferOffset, 100);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunModelIdentificationResponse
    let len;
    let data = new RunModelIdentificationResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [sampling_time]
    data.sampling_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [samples]
    data.samples = _arrayDeserializer.float64(buffer, bufferOffset, 100)
    return data;
  }

  static getMessageSize(object) {
    return 809;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/RunModelIdentificationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e73ac1d7ab01f6a97c7866cdfb45f3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    float64 sampling_time
    float64[100] samples
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunModelIdentificationResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.sampling_time !== undefined) {
      resolved.sampling_time = msg.sampling_time;
    }
    else {
      resolved.sampling_time = 0.0
    }

    if (msg.samples !== undefined) {
      resolved.samples = msg.samples;
    }
    else {
      resolved.samples = new Array(100).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: RunModelIdentificationRequest,
  Response: RunModelIdentificationResponse,
  md5sum() { return 'd0062710edaa1716482716b90a44c970'; },
  datatype() { return 'statek_hw/RunModelIdentification'; }
};
