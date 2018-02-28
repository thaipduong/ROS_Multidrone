#!/usr/bin/env python

PKG = 'drone'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import math
import numpy
import random
import time
import json
from drone.msg import NavSatFix
from drone.msg import GasSensorData
from drone.msg import DroneComm
from sensor_msgs.msg import NavSatFix

def Distance(triplet1, triplet2):
    (x1,y1,z1) = triplet1
    (x2,y2,z2) = triplet2
    #print "CommRangeDetector: drone 1 is (%f|%f|%f)" %(y1,x1,z1)
    #print "CommRangeDetector: drone 2 is (%f|%f|%f)" %(y2,x2,z2)
    xDist = (x1 - x2)*111111.11
    yDist = (y1 - y2)*111111.11
    zDist = (z1 - z2)
    #print "CommRangeDetector: axis dist is (%f|%f|%f) = (%f)" %(xDist,yDist,zDist, ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2))
    return ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2)

class DroneGPSSubscriber:
    def __init__(self, id):
        self.id = id
        self.GPSReceiverName = "uav%d/mavros/state" %id
        self.longitude = -1000
        self.latitude = -1000
        self.altitude = -1000
        self.sub = None
    def setSubscribe(self):
        if self.sub == None:         #topic below
            self.sub = rospy.Subscriber(self.GPSReceiverName, NavSatFix, self.GPSReceive)
    def GPSReceive(self,msg):
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        self.altitude = msg.altitude
        #rospy.loginfo("CRD: Drone %d received GPS data: location is now (%f|%f|%f) on %s" %(self.id, msg.longitude, msg.latitude, msg.altitude, self.GPSReceiverName))
    def getGPS(self):
        #rospy.loginfo("Get location issued: location is now (%f|%f|%f)" %(self.longitude, self.latitude, self.altitude))
        return (self.longitude,self.latitude,self.altitude)

class DroneCommPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("CommDetection", DroneComm, queue_size=10)
    def publishDronePair(self, d1, d2):
        msg = DroneComm()
        msg.d1 = d1
        msg.d2 = d2
        #rospy.loginfo("Drone Comm Range Detector publishing a drone pair: (%d|%d)" %(d1, d2))
        self.pub.publish(msg)

class CommRangeDetector:
    def __init__(self, numDrones, commRange):
        # Calculate explorable area grid bounds in terms of GPS locations based on input exploration area size
        #rospy.init_node('CommRangeDetector', anonymous = True)
        self.num = numDrones
        self.commRange = commRange
        self.droneGPSTriples = {}
        self.droneROSTopics = {}
        for i in range(numDrones):
            self.droneROSTopics[i+1] = DroneGPSSubscriber(i+1)
            self.droneGPSTriples[i+1] = (-1000,-1000,-1000)
            self.droneROSTopics[i+1].setSubscribe()
        self.commPublisher = DroneCommPublisher()
    
    def ProcessDroneSetGPS(self):
        for i in range(self.num):
            self.droneROSTopics[i+1].setSubscribe()
            self.droneGPSTriples[i+1] = self.droneROSTopics[i+1].getGPS()
        for i in range(self.num):
            for j in range(self.num - i - 1):
                t1 = self.droneGPSTriples[i+1]
                t2 = self.droneGPSTriples[j+i+1+1]
                dist = Distance(t1,t2)
                rospy.loginfo("CommRangeDetector: dist between %d/%d is %f" %(i+1, j+i+1+1,dist))
                if (dist < self.commRange):
                    self.commPublisher.publishDronePair(i+1,j+i+1+1)
                    rospy.loginfo("CommRangeDetector: Publishing drone pair %d/%d" %(i+1, j+i+1+1))









