;; Auto-generated. Do not edit!


(when (boundp 'camera_opencv::get_top_view)
  (if (not (find-package "CAMERA_OPENCV"))
    (make-package "CAMERA_OPENCV"))
  (shadow 'get_top_view (find-package "CAMERA_OPENCV")))
(unless (find-package "CAMERA_OPENCV::GET_TOP_VIEW")
  (make-package "CAMERA_OPENCV::GET_TOP_VIEW"))

(in-package "ROS")
;;//! \htmlinclude get_top_view.msg.html


(defclass camera_opencv::get_top_view
  :super ros::object
  :slots ())

(defmethod camera_opencv::get_top_view
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get camera_opencv::get_top_view :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get camera_opencv::get_top_view :datatype-) "camera_opencv/get_top_view")
(setf (get camera_opencv::get_top_view :definition-)
      "
")



(provide :camera_opencv/get_top_view "d41d8cd98f00b204e9800998ecf8427e")


