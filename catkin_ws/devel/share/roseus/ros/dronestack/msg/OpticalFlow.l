;; Auto-generated. Do not edit!


(when (boundp 'dronestack::OpticalFlow)
  (if (not (find-package "DRONESTACK"))
    (make-package "DRONESTACK"))
  (shadow 'OpticalFlow (find-package "DRONESTACK")))
(unless (find-package "DRONESTACK::OPTICALFLOW")
  (make-package "DRONESTACK::OPTICALFLOW"))

(in-package "ROS")
;;//! \htmlinclude OpticalFlow.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dronestack::OpticalFlow
  :super ros::object
  :slots (_header _OptF_x _OptF_y _OptF_z ))

(defmethod dronestack::OpticalFlow
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:OptF_x __OptF_x) 0.0)
    ((:OptF_y __OptF_y) 0.0)
    ((:OptF_z __OptF_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _OptF_x (float __OptF_x))
   (setq _OptF_y (float __OptF_y))
   (setq _OptF_z (float __OptF_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:OptF_x
   (&optional __OptF_x)
   (if __OptF_x (setq _OptF_x __OptF_x)) _OptF_x)
  (:OptF_y
   (&optional __OptF_y)
   (if __OptF_y (setq _OptF_y __OptF_y)) _OptF_y)
  (:OptF_z
   (&optional __OptF_z)
   (if __OptF_z (setq _OptF_z __OptF_z)) _OptF_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _OptF_x
    4
    ;; float32 _OptF_y
    4
    ;; float32 _OptF_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _OptF_x
       (sys::poke _OptF_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _OptF_y
       (sys::poke _OptF_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _OptF_z
       (sys::poke _OptF_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _OptF_x
     (setq _OptF_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _OptF_y
     (setq _OptF_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _OptF_z
     (setq _OptF_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dronestack::OpticalFlow :md5sum-) "170905b60a5f60e2e27710db96379516")
(setf (get dronestack::OpticalFlow :datatype-) "dronestack/OpticalFlow")
(setf (get dronestack::OpticalFlow :definition-)
      "# OpticalFlow.msg
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

")



(provide :dronestack/OpticalFlow "170905b60a5f60e2e27710db96379516")


