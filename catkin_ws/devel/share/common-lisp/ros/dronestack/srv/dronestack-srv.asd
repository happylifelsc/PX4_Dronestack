
(cl:in-package :asdf)

(defsystem "dronestack-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "waypoint_nav" :depends-on ("_package_waypoint_nav"))
    (:file "_package_waypoint_nav" :depends-on ("_package"))
  ))