; Auto-generated. Do not edit!


(in-package sony_eye_node-srv)


;//! \htmlinclude CameraServerMessage-request.msg.html

(defclass <CameraServerMessage-request> (ros-message)
  ((Command
    :reader Command-val
    :initarg :Command
    :type string
    :initform "")
   (EyeIndex
    :reader EyeIndex-val
    :initarg :EyeIndex
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CameraServerMessage-request>) ostream)
  "Serializes a message object of type '<CameraServerMessage-request>"
  (let ((__ros_str_len (length (slot-value msg 'Command))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'Command))
    (write-byte (ldb (byte 8 0) (slot-value msg 'EyeIndex)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'EyeIndex)) ostream)
)
(defmethod deserialize ((msg <CameraServerMessage-request>) istream)
  "Deserializes a message object of type '<CameraServerMessage-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'Command) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'Command) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'EyeIndex)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'EyeIndex)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CameraServerMessage-request>)))
  "Returns string type for a service object of type '<CameraServerMessage-request>"
  "sony_eye_node/CameraServerMessageRequest")
(defmethod md5sum ((type (eql '<CameraServerMessage-request>)))
  "Returns md5sum for a message object of type '<CameraServerMessage-request>"
  "cf979b998f326c36ce4cb0b0a4605093")
(defmethod message-definition ((type (eql '<CameraServerMessage-request>)))
  "Returns full string definition for message of type '<CameraServerMessage-request>"
  (format nil "string Command~%int16 EyeIndex~%~%"))
(defmethod serialization-length ((msg <CameraServerMessage-request>))
  (+ 0
     4 (length (slot-value msg 'Command))
     2
))
(defmethod ros-message-to-list ((msg <CameraServerMessage-request>))
  "Converts a ROS message object to a list"
  (list '<CameraServerMessage-request>
    (cons ':Command (Command-val msg))
    (cons ':EyeIndex (EyeIndex-val msg))
))
;//! \htmlinclude CameraServerMessage-response.msg.html

(defclass <CameraServerMessage-response> (ros-message)
  ((Value
    :reader Value-val
    :initarg :Value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CameraServerMessage-response>) ostream)
  "Serializes a message object of type '<CameraServerMessage-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'Value)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'Value)) ostream)
)
(defmethod deserialize ((msg <CameraServerMessage-response>) istream)
  "Deserializes a message object of type '<CameraServerMessage-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'Value)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'Value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CameraServerMessage-response>)))
  "Returns string type for a service object of type '<CameraServerMessage-response>"
  "sony_eye_node/CameraServerMessageResponse")
(defmethod md5sum ((type (eql '<CameraServerMessage-response>)))
  "Returns md5sum for a message object of type '<CameraServerMessage-response>"
  "cf979b998f326c36ce4cb0b0a4605093")
(defmethod message-definition ((type (eql '<CameraServerMessage-response>)))
  "Returns full string definition for message of type '<CameraServerMessage-response>"
  (format nil "int16 Value~%~%~%"))
(defmethod serialization-length ((msg <CameraServerMessage-response>))
  (+ 0
     2
))
(defmethod ros-message-to-list ((msg <CameraServerMessage-response>))
  "Converts a ROS message object to a list"
  (list '<CameraServerMessage-response>
    (cons ':Value (Value-val msg))
))
(defmethod service-request-type ((msg (eql 'CameraServerMessage)))
  '<CameraServerMessage-request>)
(defmethod service-response-type ((msg (eql 'CameraServerMessage)))
  '<CameraServerMessage-response>)
(defmethod ros-datatype ((msg (eql 'CameraServerMessage)))
  "Returns string type for a service object of type '<CameraServerMessage>"
  "sony_eye_node/CameraServerMessage")
