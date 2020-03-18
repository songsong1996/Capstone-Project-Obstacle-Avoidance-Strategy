; Auto-generated. Do not edit!


(cl:in-package camera_opencv-msg)


;//! \htmlinclude get_top_view.msg.html

(cl:defclass <get_top_view> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass get_top_view (<get_top_view>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_top_view>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_top_view)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_opencv-msg:<get_top_view> is deprecated: use camera_opencv-msg:get_top_view instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_top_view>) ostream)
  "Serializes a message object of type '<get_top_view>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_top_view>) istream)
  "Deserializes a message object of type '<get_top_view>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_top_view>)))
  "Returns string type for a message object of type '<get_top_view>"
  "camera_opencv/get_top_view")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_top_view)))
  "Returns string type for a message object of type 'get_top_view"
  "camera_opencv/get_top_view")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_top_view>)))
  "Returns md5sum for a message object of type '<get_top_view>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_top_view)))
  "Returns md5sum for a message object of type 'get_top_view"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_top_view>)))
  "Returns full string definition for message of type '<get_top_view>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_top_view)))
  "Returns full string definition for message of type 'get_top_view"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_top_view>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_top_view>))
  "Converts a ROS message object to a list"
  (cl:list 'get_top_view
))
