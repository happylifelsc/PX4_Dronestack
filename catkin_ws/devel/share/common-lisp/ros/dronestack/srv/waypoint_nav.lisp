; Auto-generated. Do not edit!


(cl:in-package dronestack-srv)


;//! \htmlinclude waypoint_nav-request.msg.html

(cl:defclass <waypoint_nav-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0)
   (c
    :reader c
    :initarg :c
    :type cl:float
    :initform 0.0))
)

(cl:defclass waypoint_nav-request (<waypoint_nav-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <waypoint_nav-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'waypoint_nav-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dronestack-srv:<waypoint_nav-request> is deprecated: use dronestack-srv:waypoint_nav-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <waypoint_nav-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dronestack-srv:a-val is deprecated.  Use dronestack-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <waypoint_nav-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dronestack-srv:b-val is deprecated.  Use dronestack-srv:b instead.")
  (b m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <waypoint_nav-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dronestack-srv:c-val is deprecated.  Use dronestack-srv:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <waypoint_nav-request>) ostream)
  "Serializes a message object of type '<waypoint_nav-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <waypoint_nav-request>) istream)
  "Deserializes a message object of type '<waypoint_nav-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<waypoint_nav-request>)))
  "Returns string type for a service object of type '<waypoint_nav-request>"
  "dronestack/waypoint_navRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint_nav-request)))
  "Returns string type for a service object of type 'waypoint_nav-request"
  "dronestack/waypoint_navRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<waypoint_nav-request>)))
  "Returns md5sum for a message object of type '<waypoint_nav-request>"
  "522b662e714d54037c689130a0d58075")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'waypoint_nav-request)))
  "Returns md5sum for a message object of type 'waypoint_nav-request"
  "522b662e714d54037c689130a0d58075")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<waypoint_nav-request>)))
  "Returns full string definition for message of type '<waypoint_nav-request>"
  (cl:format cl:nil "float64 a~%float64 b~%float64 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'waypoint_nav-request)))
  "Returns full string definition for message of type 'waypoint_nav-request"
  (cl:format cl:nil "float64 a~%float64 b~%float64 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <waypoint_nav-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <waypoint_nav-request>))
  "Converts a ROS message object to a list"
  (cl:list 'waypoint_nav-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
    (cl:cons ':c (c msg))
))
;//! \htmlinclude waypoint_nav-response.msg.html

(cl:defclass <waypoint_nav-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass waypoint_nav-response (<waypoint_nav-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <waypoint_nav-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'waypoint_nav-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dronestack-srv:<waypoint_nav-response> is deprecated: use dronestack-srv:waypoint_nav-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <waypoint_nav-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dronestack-srv:success-val is deprecated.  Use dronestack-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <waypoint_nav-response>) ostream)
  "Serializes a message object of type '<waypoint_nav-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <waypoint_nav-response>) istream)
  "Deserializes a message object of type '<waypoint_nav-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<waypoint_nav-response>)))
  "Returns string type for a service object of type '<waypoint_nav-response>"
  "dronestack/waypoint_navResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint_nav-response)))
  "Returns string type for a service object of type 'waypoint_nav-response"
  "dronestack/waypoint_navResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<waypoint_nav-response>)))
  "Returns md5sum for a message object of type '<waypoint_nav-response>"
  "522b662e714d54037c689130a0d58075")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'waypoint_nav-response)))
  "Returns md5sum for a message object of type 'waypoint_nav-response"
  "522b662e714d54037c689130a0d58075")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<waypoint_nav-response>)))
  "Returns full string definition for message of type '<waypoint_nav-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'waypoint_nav-response)))
  "Returns full string definition for message of type 'waypoint_nav-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <waypoint_nav-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <waypoint_nav-response>))
  "Converts a ROS message object to a list"
  (cl:list 'waypoint_nav-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'waypoint_nav)))
  'waypoint_nav-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'waypoint_nav)))
  'waypoint_nav-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint_nav)))
  "Returns string type for a service object of type '<waypoint_nav>"
  "dronestack/waypoint_nav")