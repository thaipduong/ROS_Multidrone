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
import DroneModule;
import FakeDroneSensor;
import FakeDroneGPS;
import CommRangeDetector;

if __name__ == "__main__":
    rospy.init_node("dronetest", anonymous = True)

    numDrones = int(sys.argv[1])
    # Desired grid resolution, number of drones, and communication radius (drone-to-drone range)
    resolutions = {'x':40,'y':40,'z':40}

    xRange = {'min':47.3977, 'max':47.40}
    yRange = {'max':8.55, 'min':8.545}
    zRange = {'max':590, 'min':550}

    xStart = {}
    yStart = {}
    zStart = {}

    xStart[1] = -35.36326
    yStart[1] = 149.16523
    zStart[1] = 635

    xStart[2] = -35.36332
    yStart[2] = 149.16523
    zStart[2] = 635

    #~10000km per 90 degrees -> 111111.11m per degree
    print "Drone Test Script Start!"

    #Need to handle manually setting each drone's ID
    print "no droneGPS created: simulation GPS used instead"

    #Fake gas sensor module data
    #randomFlag: 0 = seeded random values, 1 = non-seeded random values
    randomFlag = 0
    gasSensors = {}
    for i in range(1,numDrones+1):
        gasSensors[i] = FakeDroneSensor.FakeDroneSensor(xRange, yRange, zRange, resolutions, randomFlag, i, 1, 3)
        print "FakeGasSensorModule w/ id=%d created" %numDrones
    print "gasSensorModule created"

    commDetector = CommRangeDetector.CommRangeDetector(2,5)
    print "commDetection created"

    drones = {}
    for i in range(1,numDrones+1):
        drones[i] = DroneModule.Drone(xRange, yRange, zRange, resolutions, i)
        print "Drone id %d created" %i
    print "Drones created"
    for i in range(1,numDrones+1):
        drones[i].PublishObj()

    #TODO: Test aging exponent in sim
    print "Starting loop!"

    loop = 1
    while (True):
        for i in range(1,numDrones+1):
            drones[i].PublishObj()
            gasSensors[i].PublishGasSensorValue()
        commDetector.ProcessDroneSetGPS()
        time.sleep(0.25)
        for i in range(1,numDrones+1):
            drones[i].UpdateMaps()
            drones[i].CheckForTx()
        time.sleep(0.25)
        for i in range(1,numDrones+1):
            drones[i].SendDataTx()
        time.sleep(0.5)
        for i in range(1,numDrones+1):
            drones[i].UpdateMapsForDataTx()
        time.sleep(0.25)
        commDetector.CheckTxCorrect(drones)
        print "=====Completed loop %d!=====" %(loop)
        loop = loop + 1
	time.sleep(0.5)





