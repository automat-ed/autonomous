;; Auto-generated. Do not edit!


(when (boundp 'sdp_pkg::Control_msg)
  (if (not (find-package "SDP_PKG"))
    (make-package "SDP_PKG"))
  (shadow 'Control_msg (find-package "SDP_PKG")))
(unless (find-package "SDP_PKG::CONTROL_MSG")
  (make-package "SDP_PKG::CONTROL_MSG"))

(in-package "ROS")
;;//! \htmlinclude Control_msg.msg.html


(defclass sdp_pkg::Control_msg
  :super ros::object
  :slots (_speed _acceleration _angle ))

(defmethod sdp_pkg::Control_msg
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:acceleration __acceleration) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _acceleration (float __acceleration))
   (setq _angle (float __angle))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; float32 _acceleration
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sdp_pkg::Control_msg :md5sum-) "8c3b384e49aad271544f4a6b6032d073")
(setf (get sdp_pkg::Control_msg :datatype-) "sdp_pkg/Control_msg")
(setf (get sdp_pkg::Control_msg :definition-)
      "# Driving command for the robot in the simulation

float32 speed
float32 acceleration
float32 angle

")



(provide :sdp_pkg/Control_msg "8c3b384e49aad271544f4a6b6032d073")


