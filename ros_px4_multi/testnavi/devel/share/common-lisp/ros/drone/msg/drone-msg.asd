
(cl:in-package :asdf)

(defsystem "drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DroneComm" :depends-on ("_package_DroneComm"))
    (:file "_package_DroneComm" :depends-on ("_package"))
    (:file "GasSensorData" :depends-on ("_package_GasSensorData"))
    (:file "_package_GasSensorData" :depends-on ("_package"))
    (:file "GridTx" :depends-on ("_package_GridTx"))
    (:file "_package_GridTx" :depends-on ("_package"))
    (:file "NavSatFix" :depends-on ("_package_NavSatFix"))
    (:file "_package_NavSatFix" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))