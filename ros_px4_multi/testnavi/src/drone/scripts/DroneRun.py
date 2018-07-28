#!/usr/bin/env python

'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                                 David ?
                                                         Date  : Jul 27 2018
                              ROS Multi-drone Sim

  File Name  : DroneRun.py
  Description: This script sets up all the ROS modules associated with running
               the drone. 
---*-----------------------------------------------------------------------*'''

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

'''[GLOBAL VARS]------------------------------------------------------------'''
COMM_DROPOFF = 25

'''[main]----------------------------------------------------------------------
  Drives program, initializes all modules and drones as well as their ROS
  counterparts.
----------------------------------------------------------------------------'''
if __name__ == "__main__":
  rospy.init_node("dronetest", anonymous = True)

  numDrones = int(sys.argv[1])
  # Desired grid resolution, number of drones, and communication radius (drone-to-drone range)
  resolutions = {'x':40,'y':40,'z':40}

  xRange = {'min':47.3975, 'max':47.3980}
  yRange = {'max':8.5455, 'min':8.545}
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
  print "[main] Drone Test Script Start!"

  #Need to handle manually setting each drone's ID
  print "[main] no droneGPS created: simulation GPS used instead"

  #Fake gas sensor module data
  #randomFlag: 0 = seeded random values, 1 = non-seeded random values
  randomFlag = 0
  gasSensors = {}
  for i in range(1,numDrones+1):
    gasSensors[i] = FakeDroneSensor.FakeDroneSensor(xRange, yRange, zRange, resolutions, randomFlag, i, 1, 3)
    print "[main] FakeGasSensorModule w/ id=%d created" %numDrones
  print "[main] gasSensorModule created"

  commDetector = CommRangeDetector.CommRangeDetector(2, COMM_DROPOFF)
  print "[main] commDetection created"

  drones = {}
  for i in range(1,numDrones+1):
    drones[i] = DroneModule.Drone(xRange, yRange, zRange, resolutions, i)
    print "[main] Drone id %d created" %i
  print "[main] Drones created"
  for i in range(1,numDrones+1):
    drones[i].PublishObj()

  #TODO: Test aging exponent in sim
  print "[main] Starting loop!"

  loop = 1
  while not rospy.is_shutdown():

    #TODO find way to parallelize the drones better
    
    '''
    for i in range(1, numDrones + 1):
      drones[i].PublishObj()
      gasSensors[i].PublishGasSensorValue()
    '''

    #publish objectives and air quality samples
    for i in range(1,numDrones+1):
      drones[i].PublishObj()
      gasSensors[i].PublishGasSensorValue()
    
    commDetector.ProcessDroneSetGPS()
    time.sleep(0.25)
    
    #update maps and check for inter-drone comms
    for i in range(1,numDrones+1):
      drones[i].UpdateMaps()
      drones[i].CheckForTx()
    
    time.sleep(0.25)

    #send inter-drone comms
    for i in range(1,numDrones+1):
      drones[i].SendDataTx()
    
    time.sleep(0.5)

    for i in range(1,numDrones+1):
      drones[i].UpdateMapsForDataTx()

    time.sleep(0.25)

    commDetector.CheckTxCorrect(drones)
    
    print "[main] [%d] Iteration complete ----------" %(loop)
    loop = loop + 1
  
  print "[main] Rospy shutdown detected. Stopped drone ROS modules!"


