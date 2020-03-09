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

class speech_configRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.language = null;
      this.pitch = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('language')) {
        this.language = initObj.language
      }
      else {
        this.language = '';
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type speech_configRequest
    // Serialize message field [language]
    bufferOffset = _serializer.string(obj.language, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.int16(obj.pitch, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.int16(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type speech_configRequest
    let len;
    let data = new speech_configRequest(null);
    // Deserialize message field [language]
    data.language = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.language.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_robot_interface/speech_configRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08a23f216ce477ab09150cf3c5975177';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string language
    int16  pitch
    int16  speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new speech_configRequest(null);
    if (msg.language !== undefined) {
      resolved.language = msg.language;
    }
    else {
      resolved.language = ''
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    return resolved;
    }
};

class speech_configResponse {
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
    // Serializes a message object of type speech_configResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type speech_configResponse
    let len;
    let data = new speech_configResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_robot_interface/speech_configResponse';
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
    const resolved = new speech_configResponse(null);
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
  Request: speech_configRequest,
  Response: speech_configResponse,
  md5sum() { return 'bb0832ad049396f7ac17944e2242235c'; },
  datatype() { return 'qt_robot_interface/speech_config'; }
};
