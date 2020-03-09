// Auto-generated. Do not edit!

// (in-package qt_motors_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.temperature = null;
      this.voltage = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = [];
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorState
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [temperature]
    bufferOffset = _arraySerializer.float32(obj.temperature, buffer, bufferOffset, null);
    // Serialize message field [voltage]
    bufferOffset = _arraySerializer.float32(obj.voltage, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorState
    let len;
    let data = new MotorState(null);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [temperature]
    data.temperature = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [voltage]
    data.voltage = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 4 * object.temperature.length;
    length += 4 * object.voltage.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_motors_controller/MotorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c8e09c6156fe0a406db90d8632033e1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #std_msgs/Header header
    string[] name
    float32[] temperature
    float32[] voltage
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorState(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = []
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = []
    }

    return resolved;
    }
};

module.exports = MotorState;
