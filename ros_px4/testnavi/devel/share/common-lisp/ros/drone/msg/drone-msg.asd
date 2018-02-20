
(cl:in-package :asdf)

(defsystem "drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GPS_Coord" :depends-on ("_package_GPS_Coord"))
    (:file "_package_GPS_Coord" :depends-on ("_package"))
    (:file "GasSensorData" :depends-on ("_package_GasSensorData"))
    (:file "_package_GasSensorData" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))