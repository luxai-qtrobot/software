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

class set_velocityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parts = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('parts')) {
        this.parts = initObj.parts
      }
      else {
        this.parts = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_velocityRequest
    // Serialize message field [parts]
    bufferOffset = _arraySerializer.string(obj.parts, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _serializer.uint8(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_velocityRequest
    let len;
    let data = new set_velocityRequest(null);
    // Deserialize message field [parts]
    data.parts = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _deserializer.uint8(buffer, bufferOffset);
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
    return 'qt_motors_controller/set_velocityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '580221f6422fd270b3c14b7a16c3681e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string[] parts
    uint8  velocity
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_velocityRequest(null);
    if (msg.parts !== undefined) {
      resolved.parts = msg.parts;
    }
    else {
      resolved.parts = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0
    }

    return resolved;
    }
};

class set_velocityResponse {
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
    // Serializes a message object of type set_velocityResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_velocityResponse
    let len;
    let data = new set_velocityResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_motors_controller/set_velocityResponse';
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
    const resolved = new set_velocityResponse(null);
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
  Request: set_velocityRequest,
  Response: set_velocityResponse,
  md5sum() { return '68003f66a5a441a7e064a7fc5cd19661'; },
  datatype() { return 'qt_motors_controller/set_velocity'; }
};
