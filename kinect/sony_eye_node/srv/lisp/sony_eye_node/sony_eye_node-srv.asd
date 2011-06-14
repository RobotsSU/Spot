
(in-package :asdf)

(defsystem "sony_eye_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "CameraServerMessage" :depends-on ("_package"))
    (:file "_package_CameraServerMessage" :depends-on ("_package"))
    ))
