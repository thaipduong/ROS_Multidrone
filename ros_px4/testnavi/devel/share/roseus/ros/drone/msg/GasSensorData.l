;; Auto-generated. Do not edit!


(when (boundp 'drone::GasSensorData)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'GasSensorData (find-package "DRONE")))
(unless (find-package "DRONE::GASSENSORDATA")
  (make-package "DRONE::GASSENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude GasSensorData.msg.html


(defclass drone::GasSensorData
  :super ros::object
  :slots (_value ))

(defmethod drone::GasSensorData
  (:init
   (&key
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _value (float __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get drone::GasSensorData :md5sum-) "1b1594d2b74931ef8fe7be8e2d594455")
(setf (get drone::GasSensorData :datatype-) "drone/GasSensorData")
(setf (get drone::GasSensorData :definition-)
      "float64 value

")



(provide :drone/GasSensorData "1b1594d2b74931ef8fe7be8e2d594455")


