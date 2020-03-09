// Auto-generated. Do not edit!

// (in-package qt_gesture_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class gesture_recordRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parts = null;
      this.idleParts = null;
      this.wait = null;
      this.timeout = null;
    }
    else {
      if (initObj.hasOwnProperty('parts')) {
        this.parts = initObj.parts
      }
      else {
        this.parts = [];
      }
      if (initObj.hasOwnProperty('idleParts')) {
        this.idleParts = initObj.idleParts
      }
      else {
        this.idleParts = false;
      }
      if (initObj.hasOwnProperty('wait')) {
        this.wait = initObj.wait
      }
      else {
        this.wait = 0;
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gesture_recordRequest
    // Serialize message field [parts]
    bufferOffset = _arraySerializer.string(obj.parts, buffer, bufferOffset, null);
    // Serialize message field [idleParts]
    bufferOffset = _serializer.bool(obj.idleParts, buffer, bufferOffset);
    // Serialize message field [wait]
    bufferOffset = _serializer.uint8(obj.wait, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.uint8(obj.timeout, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gesture_recordRequest
    let len;
    let data = new gesture_recordRequest(null);
    // Deserialize message field [parts]
    data.parts = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [idleParts]
    data.idleParts = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [wait]
    data.wait = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.parts.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 7;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_gesture_controller/gesture_recordRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7b28f524f349bbb7a5f69f2378308888';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string[] parts
    
    
    bool idleParts
    
    
    uint8 wait
    
    
    uint8 timeout
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gesture_recordRequest(null);
    if (msg.parts !== undefined) {
      resolved.parts = msg.parts;
    }
    else {
      resolved.parts = []
    }

    if (msg.idleParts !== undefined) {
      resolved.idleParts = msg.idleParts;
    }
    else {
      resolved.idleParts = false
    }

    if (msg.wait !== undefined) {
      resolved.wait = msg.wait;
    }
    else {
      resolved.wait = 0
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = 0
    }

    return resolved;
    }
};

class gesture_recordResponse {
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
    // Serializes a message object of type gesture_recordResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gesture_recordResponse
    let len;
    let data = new gesture_recordResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_gesture_controller/gesture_recordResponse';
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
    const resolved = new gesture_recordResponse(null);
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
  Request: gesture_recordRequest,
  Response: gesture_recordResponse,
  md5sum() { return 'f920c6a17216e8fe1e02e50dada8c9d0'; },
  datatype() { return 'qt_gesture_controller/gesture_record'; }
};
