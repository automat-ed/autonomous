; Auto-generated. Do not edit!


(cl:in-package sdp_pkg-msg)


;//! \htmlinclude Control_msg.msg.html

(cl:defclass <Control_msg> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Control_msg (<Control_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sdp_pkg-msg:<Control_msg> is deprecated: use sdp_pkg-msg:Control_msg instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Control_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdp_pkg-msg:speed-val is deprecated.  Use sdp_pkg-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <Control_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdp_pkg-msg:acceleration-val is deprecated.  Use sdp_pkg-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Control_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdp_pkg-msg:angle-val is deprecated.  Use sdp_pkg-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control_msg>) ostream)
  "Serializes a message object of type '<Control_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control_msg>) istream)
  "Deserializes a message object of type '<Control_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control_msg>)))
  "Returns string type for a message object of type '<Control_msg>"
  "sdp_pkg/Control_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control_msg)))
  "Returns string type for a message object of type 'Control_msg"
  "sdp_pkg/Control_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control_msg>)))
  "Returns md5sum for a message object of type '<Control_msg>"
  "8c3b384e49aad271544f4a6b6032d073")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control_msg)))
  "Returns md5sum for a message object of type 'Control_msg"
  "8c3b384e49aad271544f4a6b6032d073")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control_msg>)))
  "Returns full string definition for message of type '<Control_msg>"
  (cl:format cl:nil "# Driving command for the robot in the simulation~%~%float32 speed~%float32 acceleration~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control_msg)))
  "Returns full string definition for message of type 'Control_msg"
  (cl:format cl:nil "# Driving command for the robot in the simulation~%~%float32 speed~%float32 acceleration~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control_msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Control_msg
    (cl:cons ':speed (speed msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':angle (angle msg))
))
