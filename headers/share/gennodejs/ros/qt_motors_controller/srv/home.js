// Auto-generated. Do not edit!

// (in-package qt_motors_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class homeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parts = null;
    }
    else {
      if (initObj.hasOwnProperty('parts')) {
        this.parts = initObj.parts
      }
      else {
        this.parts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type homeRequest
    // Serialize message field [parts]
    bufferOffset = _arraySerializer.string(obj.parts, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type homeRequest
    let len;
    let data = new homeRequest(null);
    // Deserialize message field [parts]
    data.parts = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.parts.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_motors_controller/homeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61920c0acc31cd7d2b61692a8f73f78b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string[] parts
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new homeRequest(null);
    if (msg.parts !== undefined) {
      resolved.parts = msg.parts;
    }
    else {
      resolved.parts = []
    }

    return resolved;
    }
};

class homeResponse {
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
    // Serializes a message object of type homeResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type homeResponse
    let len;
    let data = new homeResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_motors_controller/homeResponse';
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
    const resolved = new homeResponse(null);
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
  Request: homeRequest,
  Response: homeResponse,
  md5sum() { return '018aa38faa01fecffa7a4eb6cad48ceb'; },
  datatype() { return 'qt_motors_controller/home'; }
};
