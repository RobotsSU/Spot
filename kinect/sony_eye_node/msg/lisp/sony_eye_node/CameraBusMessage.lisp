; Auto-generated. Do not edit!


(in-package sony_eye_node-msg)


;//! \htmlinclude CameraBusMessage.msg.html

(defclass <CameraBusMessage> (ros-message)
  ((CameraTest
    :reader CameraTest-val
    :initarg :CameraTest
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CameraBusMessage>) ostream)
  "Serializes a message object of type '<CameraBusMessage>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'CameraTest)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'CameraTest)) ostream)
)
(defmethod deserialize ((msg <CameraBusMessage>) istream)
  "Deserializes a message object of type '<CameraBusMessage>"
  (setf (ldb (byte 8 0) (slot-value msg 'CameraTest)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'CameraTest)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CameraBusMessage>)))
  "Returns string type for a message object of type '<CameraBusMessage>"
  "sony_eye_node/CameraBusMessage")
(defmethod md5sum ((type (eql '<CameraBusMessage>)))
  "Returns md5sum for a message object of type '<CameraBusMessage>"
  "9e24ec5678a222cf183ee661af963421")
(defmethod message-definition ((type (eql '<CameraBusMessage>)))
  "Returns full string definition for message of type '<CameraBusMessage>"
  (format nil "int16 CameraTest~%~%~%~%"))
(defmethod serialization-length ((msg <CameraBusMessage>))
  (+ 0
     2
))
(defmethod ros-message-to-list ((msg <CameraBusMessage>))
  "Converts a ROS message object to a list"
  (list '<CameraBusMessage>
    (cons ':CameraTest (CameraTest-val msg))
))
