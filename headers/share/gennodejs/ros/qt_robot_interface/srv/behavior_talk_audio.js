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

class behavior_talk_audioRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.filename = null;
      this.filepath = null;
    }
    else {
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = '';
      }
      if (initObj.hasOwnProperty('filepath')) {
        this.filepath = initObj.filepath
      }
      else {
        this.filepath = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type behavior_talk_audioRequest
    // Serialize message field [filename]
    bufferOffset = _serializer.string(obj.filename, buffer, bufferOffset);
    // Serialize message field [filepath]
    bufferOffset = _serializer.string(obj.filepath, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type behavior_talk_audioRequest
    let len;
    let data = new behavior_talk_audioRequest(null);
    // Deserialize message field [filename]
    data.filename = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [filepath]
    data.filepath = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.filename.length;
    length += object.filepath.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_robot_interface/behavior_talk_audioRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49f2230aca62473845f7a546269c1759';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string filename
    string filepath
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new behavior_talk_audioRequest(null);
    if (msg.filename !== undefined) {
      resolved.filename = msg.filename;
    }
    else {
      resolved.filename = ''
    }

    if (msg.filepath !== undefined) {
      resolved.filepath = msg.filepath;
    }
    else {
      resolved.filepath = ''
    }

    return resolved;
    }
};

class behavior_talk_audioResponse {
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
    // Serializes a message object of type behavior_talk_audioResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type behavior_talk_audioResponse
    let len;
    let data = new behavior_talk_audioResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_robot_interface/behavior_talk_audioResponse';
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
    const resolved = new behavior_talk_audioResponse(null);
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
  Request: behavior_talk_audioRequest,
  Response: behavior_talk_audioResponse,
  md5sum() { return '52268a262dc8cf03d5103995c9714ffa'; },
  datatype() { return 'qt_robot_interface/behavior_talk_audio'; }
};
