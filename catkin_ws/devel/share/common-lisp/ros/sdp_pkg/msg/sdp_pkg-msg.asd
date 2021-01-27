
(cl:in-package :asdf)

(defsystem "sdp_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Control_msg" :depends-on ("_package_Control_msg"))
    (:file "_package_Control_msg" :depends-on ("_package"))
  ))