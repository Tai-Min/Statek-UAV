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

class SetImuParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.imu_update_rate_ms = null;
      this.acc_bias = null;
      this.gyro_bias = null;
      this.mag_bias = null;
      this.mag_scale = null;
      this.mag_dec_deg = null;
      this.mag_dec_min = null;
      this.mag_dec_sec = null;
    }
    else {
      if (initObj.hasOwnProperty('imu_update_rate_ms')) {
        this.imu_update_rate_ms = initObj.imu_update_rate_ms
      }
      else {
        this.imu_update_rate_ms = 0;
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
      if (initObj.hasOwnProperty('mag_dec_deg')) {
        this.mag_dec_deg = initObj.mag_dec_deg
      }
      else {
        this.mag_dec_deg = 0.0;
      }
      if (initObj.hasOwnProperty('mag_dec_min')) {
        this.mag_dec_min = initObj.mag_dec_min
      }
      else {
        this.mag_dec_min = 0.0;
      }
      if (initObj.hasOwnProperty('mag_dec_sec')) {
        this.mag_dec_sec = initObj.mag_dec_sec
      }
      else {
        this.mag_dec_sec = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetImuParamsRequest
    // Serialize message field [imu_update_rate_ms]
    bufferOffset = _serializer.uint32(obj.imu_update_rate_ms, buffer, bufferOffset);
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
    // Serialize message field [mag_dec_deg]
    bufferOffset = _serializer.float64(obj.mag_dec_deg, buffer, bufferOffset);
    // Serialize message field [mag_dec_min]
    bufferOffset = _serializer.float64(obj.mag_dec_min, buffer, bufferOffset);
    // Serialize message field [mag_dec_sec]
    bufferOffset = _serializer.float64(obj.mag_dec_sec, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetImuParamsRequest
    let len;
    let data = new SetImuParamsRequest(null);
    // Deserialize message field [imu_update_rate_ms]
    data.imu_update_rate_ms = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [acc_bias]
    data.acc_bias = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [gyro_bias]
    data.gyro_bias = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mag_bias]
    data.mag_bias = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mag_scale]
    data.mag_scale = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mag_dec_deg]
    data.mag_dec_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mag_dec_min]
    data.mag_dec_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mag_dec_sec]
    data.mag_dec_sec = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 124;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/SetImuParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa4190713a3de088f6d4ed82233d3a5a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 imu_update_rate_ms
    float64[3] acc_bias # Some value from calibrated MPU9250.
    float64[3] gyro_bias # Some value from calibrated MPU9250.
    float64[3] mag_bias # Some value from calibrated MPU9250.
    float64[3] mag_scale # Some value from calibrated MPU9250.
    float64 mag_dec_deg # From internet or whatever.
    float64 mag_dec_min
    float64 mag_dec_sec
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetImuParamsRequest(null);
    if (msg.imu_update_rate_ms !== undefined) {
      resolved.imu_update_rate_ms = msg.imu_update_rate_ms;
    }
    else {
      resolved.imu_update_rate_ms = 0
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

    if (msg.mag_dec_deg !== undefined) {
      resolved.mag_dec_deg = msg.mag_dec_deg;
    }
    else {
      resolved.mag_dec_deg = 0.0
    }

    if (msg.mag_dec_min !== undefined) {
      resolved.mag_dec_min = msg.mag_dec_min;
    }
    else {
      resolved.mag_dec_min = 0.0
    }

    if (msg.mag_dec_sec !== undefined) {
      resolved.mag_dec_sec = msg.mag_dec_sec;
    }
    else {
      resolved.mag_dec_sec = 0.0
    }

    return resolved;
    }
};

class SetImuParamsResponse {
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
    // Serializes a message object of type SetImuParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetImuParamsResponse
    let len;
    let data = new SetImuParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'statek_hw/SetImuParamsResponse';
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
    const resolved = new SetImuParamsResponse(null);
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
  Request: SetImuParamsRequest,
  Response: SetImuParamsResponse,
  md5sum() { return 'fc6fb75dba4829e04baa420f42b24adc'; },
  datatype() { return 'statek_hw/SetImuParams'; }
};
