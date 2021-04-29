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

class RunImuCalibrationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunImuCalibrationRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunImuCalibrationRequest
    let len;
    let data = new RunImuCalibrationRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/RunImuCalibrationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunImuCalibrationRequest(null);
    return resolved;
    }
};

class RunImuCalibrationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.acc_bias = null;
      this.gyro_bias = null;
      this.mag_bias = null;
      this.mag_scale = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('acc_bias')) {
        this.acc_bias = initObj.acc_bias
      }
      else {
        this.acc_bias = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyro_bias')) {
        this.gyro_bias = initObj.gyro_bias
      }
      else {
        this.gyro_bias = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mag_bias')) {
        this.mag_bias = initObj.mag_bias
      }
      else {
        this.mag_bias = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mag_scale')) {
        this.mag_scale = initObj.mag_scale
      }
      else {
        this.mag_scale = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunImuCalibrationResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Check that the constant length array field [acc_bias] has the right length
    if (obj.acc_bias.length !== 3) {
      throw new Error('Unable to serialize array field acc_bias - length must be 3')
    }
    // Serialize message field [acc_bias]
    bufferOffset = _arraySerializer.float64(obj.acc_bias, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyro_bias] has the right length
    if (obj.gyro_bias.length !== 3) {
      throw new Error('Unable to serialize array field gyro_bias - length must be 3')
    }
    // Serialize message field [gyro_bias]
    bufferOffset = _arraySerializer.float64(obj.gyro_bias, buffer, bufferOffset, 3);
    // Check that the constant length array field [mag_bias] has the right length
    if (obj.mag_bias.length !== 3) {
      throw new Error('Unable to serialize array field mag_bias - length must be 3')
    }
    // Serialize message field [mag_bias]
    bufferOffset = _arraySerializer.float64(obj.mag_bias, buffer, bufferOffset, 3);
    // Check that the constant length array field [mag_scale] has the right length
    if (obj.mag_scale.length !== 3) {
      throw new Error('Unable to serialize array field mag_scale - length must be 3')
    }
    // Serialize message field [mag_scale]
    bufferOffset = _arraySerializer.float64(obj.mag_scale, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunImuCalibrationResponse
    let len;
    let data = new RunImuCalibrationResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [acc_bias]
    data.acc_bias = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [gyro_bias]
    data.gyro_bias = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mag_bias]
    data.mag_bias = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mag_scale]
    data.mag_scale = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 97;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/RunImuCalibrationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6cd60c1db4de1bdcd039c0b56b7d1d09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success # Set to true on success
    float64[3] acc_bias # Some value from calibrated MPU9250.
    float64[3] gyro_bias # Some value from calibrated MPU9250.
    float64[3] mag_bias # Some value from calibrated MPU9250.
    float64[3] mag_scale # Some value from calibrated MPU9250.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunImuCalibrationResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.acc_bias !== undefined) {
      resolved.acc_bias = msg.acc_bias;
    }
    else {
      resolved.acc_bias = new Array(3).fill(0)
    }

    if (msg.gyro_bias !== undefined) {
      resolved.gyro_bias = msg.gyro_bias;
    }
    else {
      resolved.gyro_bias = new Array(3).fill(0)
    }

    if (msg.mag_bias !== undefined) {
      resolved.mag_bias = msg.mag_bias;
    }
    else {
      resolved.mag_bias = new Array(3).fill(0)
    }

    if (msg.mag_scale !== undefined) {
      resolved.mag_scale = msg.mag_scale;
    }
    else {
      resolved.mag_scale = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: RunImuCalibrationRequest,
  Response: RunImuCalibrationResponse,
  md5sum() { return '6cd60c1db4de1bdcd039c0b56b7d1d09'; },
  datatype() { return 'statek_hw/RunImuCalibration'; }
};
