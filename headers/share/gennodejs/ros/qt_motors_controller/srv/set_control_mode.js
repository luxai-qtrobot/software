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

class set_control_modeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parts = null;
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('parts')) {
        this.parts = initObj.parts
      }
      else {
        this.parts = [];
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_control_modeRequest
    // Serialize message field [parts]
    bufferOffset = _arraySerializer.string(obj.parts, buffer, bufferOffset, null);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_control_modeRequest
    let len;
    let data = new set_control_modeRequest(null);
    // Deserialize message field [parts]
    data.parts = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.parts.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_motors_controller/set_control_modeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54b160c35442a5d4531dbe67c76c84be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint8 M_ON=0
    uint8 M_OFF=1
    uint8 M_BREAK=2
    
    
    string[] parts
    uint8  mode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_control_modeRequest(null);
    if (msg.parts !== undefined) {
      resolved.parts = msg.parts;
    }
    else {
      resolved.parts = []
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

// Constants for message
set_control_modeRequest.Constants = {
  M_ON: 0,
  M_OFF: 1,
  M_BREAK: 2,
}

class set_control_modeResponse {
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
    // Serializes a message object of type set_control_modeResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_control_modeResponse
    let len;
    let data = new set_control_modeResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_motors_controller/set_control_modeResponse';
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
    const resolved = new set_control_modeResponse(null);
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
  Request: set_control_modeRequest,
  Response: set_control_modeResponse,
  md5sum() { return '29ec7411588be56f1e5fa3f64ddace43'; },
  datatype() { return 'qt_motors_controller/set_control_mode'; }
};
