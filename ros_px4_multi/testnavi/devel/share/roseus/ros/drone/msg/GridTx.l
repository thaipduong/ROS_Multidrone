;; Auto-generated. Do not edit!


(when (boundp 'drone::GridTx)
  (if (not (find-package "DRONE"))
    (make-package "DRONE"))
  (shadow 'GridTx (find-package "DRONE")))
(unless (find-package "DRONE::GRIDTX")
  (make-package "DRONE::GRIDTX"))

(in-package "ROS")
;;//! \htmlinclude GridTx.msg.html


(defclass drone::GridTx
  :super ros::object
  :slots (_d1 _d2 _data ))

(defmethod drone::GridTx
  (:init
   (&key
    ((:d1 __d1) 0)
    ((:d2 __d2) 0)
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _d1 (round __d1))
   (setq _d2 (round __d2))
   (setq _data (round __data))
   self)
  (:d1
   (&optional __d1)
   (if __d1 (setq _d1 __d1)) _d1)
  (:d2
   (&optional __d2)
   (if __d2 (setq _d2 __d2)) _d2)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int64 _d1
    8
    ;; int64 _d2
    8
    ;; int64 _data
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
     ;; int64 _data
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _data (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _data) (= (length (_data . bv)) 2)) ;; bignum
              (write-long (ash (elt (_data . bv) 0) 0) s)
              (write-long (ash (elt (_data . bv) 1) -1) s))
             ((and (class _data) (= (length (_data . bv)) 1)) ;; big1
              (write-long (elt (_data . bv) 0) s)
              (write-long (if (>= _data 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _data s)(write-long (if (>= _data 0) 0 #xffffffff) s)))
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
   ;; int64 _data
#+(or :alpha :irix6 :x86_64)
      (setf _data (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _data (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get drone::GridTx :md5sum-) "e305bbdd6154480a44d99b1135a2c2dd")
(setf (get drone::GridTx :datatype-) "drone/GridTx")
(setf (get drone::GridTx :definition-)
      "int64 d1
int64 d2
int64 data

")



(provide :drone/GridTx "e305bbdd6154480a44d99b1135a2c2dd")


