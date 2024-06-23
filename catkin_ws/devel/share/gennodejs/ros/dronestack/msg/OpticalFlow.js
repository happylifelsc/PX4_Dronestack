// Auto-generated. Do not edit!

// (in-package dronestack.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OpticalFlow {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.OptF_x = null;
      this.OptF_y = null;
      this.OptF_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('OptF_x')) {
        this.OptF_x = initObj.OptF_x
      }
      else {
        this.OptF_x = 0.0;
      }
      if (initObj.hasOwnProperty('OptF_y')) {
        this.OptF_y = initObj.OptF_y
      }
      else {
        this.OptF_y = 0.0;
      }
      if (initObj.hasOwnProperty('OptF_z')) {
        this.OptF_z = initObj.OptF_z
      }
      else {
        this.OptF_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OpticalFlow
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [OptF_x]
    bufferOffset = _serializer.float32(obj.OptF_x, buffer, bufferOffset);
    // Serialize message field [OptF_y]
    bufferOffset = _serializer.float32(obj.OptF_y, buffer, bufferOffset);
    // Serialize message field [OptF_z]
    bufferOffset = _serializer.float32(obj.OptF_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OpticalFlow
    let len;
    let data = new OpticalFlow(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [OptF_x]
    data.OptF_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [OptF_y]
    data.OptF_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [OptF_z]
    data.OptF_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dronestack/OpticalFlow';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '170905b60a5f60e2e27710db96379516';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # OpticalFlow.msg
    Header header
    float32 OptF_x
    float32 OptF_y
    float32 OptF_z
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OpticalFlow(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.OptF_x !== undefined) {
      resolved.OptF_x = msg.OptF_x;
    }
    else {
      resolved.OptF_x = 0.0
    }

    if (msg.OptF_y !== undefined) {
      resolved.OptF_y = msg.OptF_y;
    }
    else {
      resolved.OptF_y = 0.0
    }

    if (msg.OptF_z !== undefined) {
      resolved.OptF_z = msg.OptF_z;
    }
    else {
      resolved.OptF_z = 0.0
    }

    return resolved;
    }
};

module.exports = OpticalFlow;
