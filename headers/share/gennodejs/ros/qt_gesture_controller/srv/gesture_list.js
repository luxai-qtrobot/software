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

class gesture_listRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gesture_listRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gesture_listRequest
    let len;
    let data = new gesture_listRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_gesture_controller/gesture_listRequest';
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
    const resolved = new gesture_listRequest(null);
    return resolved;
    }
};

class gesture_listResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gestures = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('gestures')) {
        this.gestures = initObj.gestures
      }
      else {
        this.gestures = [];
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gesture_listResponse
    // Serialize message field [gestures]
    bufferOffset = _arraySerializer.string(obj.gestures, buffer, bufferOffset, null);
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gesture_listResponse
    let len;
    let data = new gesture_listResponse(null);
    // Deserialize message field [gestures]
    data.gestures = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gestures.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qt_gesture_controller/gesture_listResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fff31f89d10c3103fdbdb3a5212feb94';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] gestures
    bool status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gesture_listResponse(null);
    if (msg.gestures !== undefined) {
      resolved.gestures = msg.gestures;
    }
    else {
      resolved.gestures = []
    }

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
  Request: gesture_listRequest,
  Response: gesture_listResponse,
  md5sum() { return 'fff31f89d10c3103fdbdb3a5212feb94'; },
  datatype() { return 'qt_gesture_controller/gesture_list'; }
};
