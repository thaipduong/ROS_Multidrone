;; Auto-generated. Do not edit!


(when (boundp 'drone::DroneComm)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'DroneComm (find-package "DRONE")))
(unless (find-package "DRONE::DRONECOMM")
  (make-package "DRONE::DRONECOMM"))

(in-package "ROS")
;;//! \htmlinclude DroneComm.msg.html


(defclass drone::DroneComm
  :super ros::object
  :slots (_d1 _d2 ))

(defmethod drone::DroneComm
  (:init
   (&key
    ((:d1 __d1) 0)
    ((:d2 __d2) 0)
    )
   (send-super :init)
   (setq _d1 (round __d1))
   (setq _d2 (round __d2))
   self)
  (:d1
   (&optional __d1)
   (if __d1 (setq _d1 __d1)) _d1)
  (:d2
   (&optional __d2)
   (if __d2 (setq _d2 __d2)) _d2)
  (:serialization-length
   ()
   (+
    ;; int64 _d1
    8
    ;; int64 _d2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _d1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _d1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _d1) (= (length (_d1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_d1 . bv) 0) 0) s)
              (write-long (ash (elt (_d1 . bv) 1) -1) s))
             ((and (class _d1) (= (length (_d1 . bv)) 1)) ;; big1
              (write-long (elt (_d1 . bv) 0) s)
              (write-long (if (>= _d1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _d1 s)(write-long (if (>= _d1 0) 0 #xffffffff) s)))
     ;; int64 _d2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _d2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _d2) (= (length (_d2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_d2 . bv) 0) 0) s)
              (write-long (ash (elt (_d2 . bv) 1) -1) s))
             ((and (class _d2) (= (length (_d2 . bv)) 1)) ;; big1
              (write-long (elt (_d2 . bv) 0) s)
              (write-long (if (>= _d2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _d2 s)(write-long (if (>= _d2 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _d1
#+(or :alpha :irix6 :x86_64)
      (setf _d1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _d1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _d2
#+(or :alpha :irix6 :x86_64)
      (setf _d2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _d2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get drone::DroneComm :md5sum-) "6709d43b931cab88bf24bfce91f3f579")
(setf (get drone::DroneComm :datatype-) "drone/DroneComm")
(setf (get drone::DroneComm :definition-)
      "int64 d1
int64 d2

")



(provide :drone/DroneComm "6709d43b931cab88bf24bfce91f3f579")


