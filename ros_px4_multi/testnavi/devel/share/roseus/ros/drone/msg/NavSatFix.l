;; Auto-generated. Do not edit!


(when (boundp 'drone::NavSatFix)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'NavSatFix (find-package "DRONE")))
(unless (find-package "DRONE::NAVSATFIX")
  (make-package "DRONE::NAVSATFIX"))

(in-package "ROS")
;;//! \htmlinclude NavSatFix.msg.html


(defclass drone::NavSatFix
  :super ros::object
  :slots (_longitude _latitude _altitude ))

(defmethod drone::NavSatFix
  (:init
   (&key
    ((:longitude __longitude) 0.0)
    ((:latitude __latitude) 0.0)
    ((:altitude __altitude) 0.0)
    )
   (send-super :init)
   (setq _longitude (float __longitude))
   (setq _latitude (float __latitude))
   (setq _altitude (float __altitude))
   self)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:serialization-length
   ()
   (+
    ;; float64 _longitude
    8
    ;; float64 _latitude
    8
    ;; float64 _altitude
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
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get drone::NavSatFix :md5sum-) "c6ce0a2191bc628a283cd615ab04a4d1")
(setf (get drone::NavSatFix :datatype-) "drone/NavSatFix")
(setf (get drone::NavSatFix :definition-)
      "float64 longitude
float64 latitude
float64 altitude

")



(provide :drone/NavSatFix "c6ce0a2191bc628a283cd615ab04a4d1")


