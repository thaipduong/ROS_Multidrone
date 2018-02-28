// Auto-generated. Do not edit!

// (in-package drone.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GridTx {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.d1 = null;
      this.d2 = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('d1')) {
        this.d1 = initObj.d1
      }
      else {
        this.d1 = 0;
      }
      if (initObj.hasOwnProperty('d2')) {
        this.d2 = initObj.d2
      }
      else {
        this.d2 = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GridTx
    // Serialize message field [d1]
    bufferOffset = _serializer.int64(obj.d1, buffer, bufferOffset);
    // Serialize message field [d2]
    bufferOffset = _serializer.int64(obj.d2, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _serializer.int64(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GridTx
    let len;
    let data = new GridTx(null);
    // Deserialize message field [d1]
    data.d1 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [d2]
    data.d2 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone/GridTx';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e305bbdd6154480a44d99b1135a2c2dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 d1
    int64 d2
    int64 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GridTx(null);
    if (msg.d1 !== undefined) {
      resolved.d1 = msg.d1;
    }
    else {
      resolved.d1 = 0
    }

    if (msg.d2 !== undefined) {
      resolved.d2 = msg.d2;
    }
    else {
      resolved.d2 = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    return resolved;
    }
};

module.exports = GridTx;
