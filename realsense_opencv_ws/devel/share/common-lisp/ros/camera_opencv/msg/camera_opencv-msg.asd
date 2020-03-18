
(cl:in-package :asdf)

(defsystem "camera_opencv-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "get_top_view" :depends-on ("_package_get_top_view"))
    (:file "_package_get_top_view" :depends-on ("_package"))
  ))