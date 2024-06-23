
(cl:in-package :asdf)

(defsystem "dronestack-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OpticalFlow" :depends-on ("_package_OpticalFlow"))
    (:file "_package_OpticalFlow" :depends-on ("_package"))
  ))