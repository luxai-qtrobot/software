// Auto-generated. Do not edit!

// (in-package qt_robot_interface.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class setting_setVolumeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.volume = null;
    }
    else {
      if (initObj.hasOwnProperty('volume')) {
        this.volume = initObj.volume
      }
      else {
        this.volume = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setting_setVolumeRequest
    // Serialize message field [volume]
    bufferOffset = _serializer.uint8(obj.volume, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setting_setVolumeRequest
    let len;
    let data = new setting_setVolumeRequest(null);
    // Deserialize message field [volume]
    data.volume = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_robot_interface/setting_setVolumeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a86acf73473ef36a9dea539b67321f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 volume
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setting_setVolumeRequest(null);
    if (msg.volume !== undefined) {
      resolved.volume = msg.volume;
    }
    else {
      resolved.volume = 0
    }

    return resolved;
    }
};

class setting_setVolumeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setting_setVolumeResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setting_setVolumeResponse
    let len;
    let data = new setting_setVolumeResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_robot_interface/setting_setVolumeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1255d4d998bd4d6585c64639b5ee9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setting_setVolumeResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: setting_setVolumeRequest,
  Response: setting_setVolumeResponse,
  md5sum() { return '14f0bfd003d9dc3318a211b307c7e7ab'; },
  datatype() { return 'qt_robot_interface/setting_setVolume'; }
};
