;; Auto-generated. Do not edit!


(when (boundp 'drone::GPS_Coord)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'GPS_Coord (find-package "DRONE")))
(unless (find-package "DRONE::GPS_COORD")
  (make-package "DRONE::GPS_COORD"))

(in-package "ROS")
;;//! \htmlinclude GPS_Coord.msg.html


(defclass drone::GPS_Coord
  :super ros::object
  :slots (_longitude _latitude _height ))

(defmethod drone::GPS_Coord
  (:init
   (&key
    ((:longitude __longitude) 0.0)
    ((:latitude __latitude) 0.0)
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _longitude (float __longitude))
   (setq _latitude (float __latitude))
   (setq _height (float __height))
   self)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; float64 _longitude
    8
    ;; float64 _latitude
    8
    ;; float64 _height
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get drone::GPS_Coord :md5sum-) "e62768fc20577eb4d6831142c7af4bea")
(setf (get drone::GPS_Coord :datatype-) "drone/GPS_Coord")
(setf (get drone::GPS_Coord :definition-)
      "float64 longitude
float64 latitude
float64 height

")



(provide :drone/GPS_Coord "e62768fc20577eb4d6831142c7af4bea")


