
(in-package :asdf)

(defsystem "sony_eye_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "CameraBusMessage" :depends-on ("_package"))
    (:file "_package_CameraBusMessage" :depends-on ("_package"))
    ))
