;; Auto-generated. Do not edit!


(when (boundp 'dronestack::waypoint_nav)
  (if (not (find-package "DRONESTACK"))
    (make-package "DRONESTACK"))
  (shadow 'waypoint_nav (find-package "DRONESTACK")))
(unless (find-package "DRONESTACK::WAYPOINT_NAV")
  (make-package "DRONESTACK::WAYPOINT_NAV"))
(unless (find-package "DRONESTACK::WAYPOINT_NAVREQUEST")
  (make-package "DRONESTACK::WAYPOINT_NAVREQUEST"))
(unless (find-package "DRONESTACK::WAYPOINT_NAVRESPONSE")
  (make-package "DRONESTACK::WAYPOINT_NAVRESPONSE"))

(in-package "ROS")





(defclass dronestack::waypoint_navRequest
  :super ros::object
  :slots (_a _b _c ))

(defmethod dronestack::waypoint_navRequest
  (:init
   (&key
    ((:a __a) 0.0)
    ((:b __b) 0.0)
    ((:c __c) 0.0)
    )
   (send-super :init)
   (setq _a (float __a))
   (setq _b (float __b))
   (setq _c (float __c))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:serialization-length
   ()
   (+
    ;; float64 _a
    8
    ;; float64 _b
    8
    ;; float64 _c
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b
       (sys::poke _b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _c
       (sys::poke _c (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b
     (setq _b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _c
     (setq _c (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dronestack::waypoint_navResponse
  :super ros::object
  :slots (_success ))

(defmethod dronestack::waypoint_navResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dronestack::waypoint_nav
  :super ros::object
  :slots ())

(setf (get dronestack::waypoint_nav :md5sum-) "522b662e714d54037c689130a0d58075")
(setf (get dronestack::waypoint_nav :datatype-) "dronestack/waypoint_nav")
(setf (get dronestack::waypoint_nav :request) dronestack::waypoint_navRequest)
(setf (get dronestack::waypoint_nav :response) dronestack::waypoint_navResponse)

(defmethod dronestack::waypoint_navRequest
  (:response () (instance dronestack::waypoint_navResponse :init)))

(setf (get dronestack::waypoint_navRequest :md5sum-) "522b662e714d54037c689130a0d58075")
(setf (get dronestack::waypoint_navRequest :datatype-) "dronestack/waypoint_navRequest")
(setf (get dronestack::waypoint_navRequest :definition-)
      "float64 a
float64 b
float64 c
---
bool success
")

(setf (get dronestack::waypoint_navResponse :md5sum-) "522b662e714d54037c689130a0d58075")
(setf (get dronestack::waypoint_navResponse :datatype-) "dronestack/waypoint_navResponse")
(setf (get dronestack::waypoint_navResponse :definition-)
      "float64 a
float64 b
float64 c
---
bool success
")



(provide :dronestack/waypoint_nav "522b662e714d54037c689130a0d58075")


