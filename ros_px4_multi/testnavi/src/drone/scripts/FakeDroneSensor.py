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

class GasSensorPublisher:
    def __init__(self, id):
        self.id = id
        self.moduleName = "FakeDroneSensor_%d" %id
        self.pub = rospy.Publisher(self.moduleName, GasSensorData, queue_size=1)
    def publishGasData(self, val):
        msg = GasSensorData()
        msg.value = val
        #rospy.loginfo("Fake sensor sending fake data: %f" %(msg.value))
        self.pub.publish(msg)

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
        #rospy.loginfo("FakeDroneSensor %d: received GPS (%f|%f|%f)" %(self.id, msg.longitude, msg.latitude, msg.altitude))
    def getGPS(self):
        #rospy.loginfo("Get location issued: location is now (%f|%f|%f)" %(self.longitude, self.latitude, self.altitude))
        return {'longitude':self.longitude,'latitude':self.latitude,'altitude':self.altitude}

class FakeDroneSensor:
    def __init__(self, xRange, yRange, zRange, resolutions, randFlag, id, numEvents, detectionRange):
        # Calculate explorable area grid bounds in terms of GPS locations based on input exploration area size
        self.id = id
        #self.moduleName = "FakeDroneSensor_%d" %id
        self.detectRange = detectionRange
        #rospy.init_node(self.moduleName, anonymous = True)
        self.GPSReceiverName = "uav%d/mavros/state" %id
        self.GPSReceiver = DroneGPSSubscriber(id)
        self.GPSReceiver.setSubscribe()
        GPSLoc = self.GPSReceiver.getGPS()
        #while (GPSLoc['latitude'] == -1000):
        #    self.GPSReceiver.setSubscribe()
        #    GPSLoc = self.GPSReceiver.getGPS()
        self.x = GPSLoc['latitude']
        self.y = GPSLoc['longitude']
        self.z = GPSLoc['altitude']
        self.xMax = xRange['max']
        self.xMin = xRange['min']
        self.yMax = yRange['max']
        self.yMin = yRange['min']
        self.zMax = zRange['max']
        self.zMin = zRange['min']
        self.bounds = {'xMax':self.xMax,'xMin':self.xMin,'yMax':self.yMax,'yMin':self.yMin,'zMin':self.zMin,'zMax':self.zMax}
        self.resolutions = resolutions
        self.sensePub = GasSensorPublisher(id)

        # Create random fake events based on exploration grid provided and number of fake events specified
        self.rand = randFlag
        if(self.rand == 0):
            random.seed(7)
        self.fakeEventNum = numEvents
        self.xNum = math.ceil( ( self.bounds['xMax'] - self.bounds['xMin'] ) / self.resolutions['x'] );
        self.yNum = math.ceil( ( self.bounds['yMax'] - self.bounds['yMin'] ) / self.resolutions['y'] );
        self.zNum = math.ceil( ( self.bounds['zMax'] - self.bounds['zMin'] ) / self.resolutions['z'] );
        self.fakeEventLocationsX = []
        self.fakeEventLocationsY = []
        self.fakeEventLocationsZ = []
        for i in range(self.fakeEventNum):
            self.fakeEventLocationsX.append((random.random()*(self.bounds['xMax'] - self.bounds['xMin']))+self.bounds['xMin'])
            self.fakeEventLocationsY.append((random.random()*(self.bounds['yMax'] - self.bounds['yMin']))+self.bounds['yMin'])
            self.fakeEventLocationsZ.append((random.random()*(self.bounds['zMax'] - self.bounds['zMin']))+self.bounds['zMin'])

    # Fake movement update for ROS testing (used with simulation GPS data)
    # Updates internal GPS location of drone 1' based on given values
    def UpdateGPS(self):
        self.GPSReceiver.setSubscribe()
        GPSLoc = self.GPSReceiver.getGPS()
        self.x = GPSLoc['latitude']
        self.y = GPSLoc['longitude']
        self.z = GPSLoc['altitude']

    # Fake sensor data value for standalone python testing
    # Returns a value based on proximity to fake locations of interest (sensing distance currently set to 20')
    def PublishGasSensorValue(self):
        val = 0
        self.UpdateGPS()
        for i in range(self.fakeEventNum):
            xDist = (self.x - self.fakeEventLocationsX[i])*111111.11
            yDist = (self.y - self.fakeEventLocationsY[i])*111111.11
            zDist = (self.z - self.fakeEventLocationsZ[i])
            temp = ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2)
            if temp < self.detectRange:
                val = val + (50/temp)
        if val > 0:
            rospy.loginfo("FakeDroneSensor %d: detected val %f, printing list of fake event locations:" %(self.id, val))
            self.PrintFakeEventLocations()
        self.sensePub.publishGasData(val)

    # Prints out fake locations of events
    def PrintFakeEventLocations(self):
        for i in range(self.fakeEventNum):
            rospy.loginfo("[%f][%f][%f]," %(self.fakeEventLocationsX[i], self.fakeEventLocationsY[i], self.fakeEventLocationsZ[i]))

