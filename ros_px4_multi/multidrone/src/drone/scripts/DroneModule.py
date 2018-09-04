#!/usr/bin/env python

'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                                 David ?
                                                         Date  : Sep 04 2018
                              ROS Multi-drone Sim

  File Name  : DroneModule.py
  Description: Handles individual drone logic in simulation. Includes simulated
               sensors, like air quality and radio bitrate limitations.
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
import threading
from drone.msg import NavSatFix
from drone.msg import GasSensorData
from drone.msg import DroneComm
from drone.msg import GridData
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import NavSatFix

import FakeDroneSensor


'''[Drone]----------------------------------------------------------------------
  Drone class, has an ID, name, sensors, grid resolution, and mission params
----------------------------------------------------------------------------'''
class Drone(threading.Thread):
  # Constructor. Requires start GPS location, exploration area size (in feet), grid resolution, and id
  def __init__(self, xRange, yRange, zRange, resolutions, id):
    super(Drone, self).__init__()
    self.daemon = True
    self.id = id
    self.moduleName = "Drone_%d" %id
    #rospy.init_node(self.moduleName, anonymous = True)
    self.proximityReq = 1             # Max distance to target to consider objective location reached
    self.isExploring = True
    self.t = 0
    self.end_thread = False

    #add sensors to drone
    #randomFlag: 0 = seeded random values, 1 = non-seeded random values
    randomFlag = 0
    self.gasSensor = FakeDroneSensor.FakeDroneSensor(xRange, yRange, zRange, resolutions, randomFlag, id, 1, 3)
    self.DataCollect = DroneDataCollection(id)

    # Calculate explorable area grid bounds in terms of GPS locations based on input exploration area size
    self.bounds = {'xMax':xRange['max'],'xMin':xRange['min'],'yMax':yRange['max'],'yMin':yRange['min'],'zMax':zRange['max'],'zMin':zRange['min']}
    self.resolutions = resolutions
    self.xResolutionSize = (self.bounds['xMax'] - self.bounds['xMin'])/self.resolutions['x']
    self.yResolutionSize = (self.bounds['yMax'] - self.bounds['yMin'])/self.resolutions['y']
    self.zResolutionSize = (self.bounds['zMax'] - self.bounds['zMin'])/self.resolutions['z']
    self.Maps = DroneObjectiveMapping( self.bounds, resolutions, id )
    
    GPSLoc = self.DataCollect.GetGPS()
    self.x = GPSLoc['x']
    self.y = GPSLoc['y']
    self.z = GPSLoc['z']

    # If drone is starting within search area, set initial start destination to furthest exploration grid corner
    # If drone is not starting within search area, set objective to closest grid location
    # TODO: Better start destination
    #print "Drone %d sx: %f/%f/%f" %(self.x, self.xMin, self.xMax)
    #print "Drone %d y: %f/%f/%f" %(self.y, self.yMin, self.yMax)
    #print "Drone %d z: %f/%f/%f" %(self.z, self.zMin, self.zMax)
    if not ((self.x <= self.bounds['xMax']) and (self.x >= self.bounds['xMin'])):
      if (self.x > self.bounds['xMax']):
        self.xObjective = self.bounds['xMax']
      else:
        self.xObjective = self.bounds['xMin']
    else:
      if self.x < ((self.bounds['xMax'] - self.bounds['xMin']) / 2):
        self.xObjective = self.bounds['xMax']
      else:
        self.xObjective = self.bounds['xMin']
    if not ((self.y <= self.bounds['yMax']) and (self.y >= self.bounds['yMin'])):
      if (self.y > self.bounds['yMax']):
        self.yObjective = self.bounds['yMax']
      else:
        self.yObjective = self.bounds['yMin']
    else:
      if self.y < ((self.bounds['yMax'] - self.bounds['yMin']) / 2):
        self.yObjective = self.bounds['yMax']
      else:
        self.yObjective = self.bounds['yMin']
    if not ((self.z <= self.bounds['zMax']) and (self.z >= self.bounds['zMin'])):
      if (self.z > self.bounds['zMax']):
        self.zObjective = self.bounds['zMax']
      else:
        self.zObjective = self.bounds['zMin']
    else:
      if self.z < ((self.bounds['zMax'] - self.bounds['zMin']) / 2):
        self.zObjective = self.bounds['zMax']
      else:
        self.zObjective = self.bounds['zMin']

    #self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
    obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
    [x,y,z] = self.Maps.GetCoordinateIndices(obj)
    print "[drone] [%d]: Setting start objective (%f|%f|%f) = (%d|%d|%d)" %(self.id, self.yObjective, self.xObjective, self.zObjective,x,y,z)
    self.CommModule = DroneCommModule(id,resolutions)
    self.CommModule.SetMaps(self.Maps)
    
    self.SetObj()
    self.PublishObj()

  def callback(self, msg):
    if msg == "end":
      self.end_thread = True

  # Update drone GPS location based on input measurement
  def UpdateLoc(self, measurement):
    (xOld,yOld,zOld) = (self.x,self.y,self.z)
    self.x = measurement.x
    self.y = measurement.y
    self.z = measurement.z
    self.t = measurement.t
    newM = DroneMeasurement(self.x, self.y, self.z, 0, 0)
    [x,y,z] = self.Maps.GetCoordinateIndices( newM )
    obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
    [xObj,yObj,zObj] = self.Maps.GetCoordinateIndices( obj )
    if (xOld == self.x) and (yOld == self.y) and (zOld == self.z):
      pass
      #print "[drone] [%d] [gps]: NO MOVEMENT" %(self.id)
    else:
      # Print current location, exploration status, and objective
      #if self.isExploring is True:
      #    print "[drone] [%d]: Exploring" %(self.id)
      #else:
      #    print "[drone] [%d]: NOT Exploring" %(self.id)
      pass

  # Calculates distance to objective (in meters based on GPS coordinates)
  def DistanceToObjective(self):
    xDist = (self.x - self.xObjective)*111111.11
    yDist = (self.y - self.yObjective)*111111.11
    zDist = (self.z - self.zObjective)
    return ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2)

  def IsWithinSearchArea(self):
    if ((self.x <= self.bounds['xMax'] + 0.5/111111.11) and (self.x >= self.bounds['xMin'] - 0.5/111111.11) and (self.y <= self.bounds['yMax'] + 0.5/111111.11) and (self.y >= self.bounds['yMin'] - 0.5/111111.11) and (self.z <= self.bounds['zMax'] + 0.5/111111.11) and (self.z >= self.bounds['zMin'] - 0.5/111111.11)):
      return 1
    else:
      return 0

  def GetID(self):
    return self.id

  def SendDataTx(self):
    #TODO: Is this correct if drone is currently swapping map information with another drone
    if self.CommModule.GetCommStatus() == 1:
      self.CommModule.DataTx()      

  def UpdateMapsForDataTx(self):
    if self.CommModule.GetGridTxReceivedStatus() == 1:
      print "[drone] [%d]: Received new data, updating maps, resetting status" %(self.id)
      maps = self.Maps.CombineReceivedMaps(self.CommModule)
      self.Maps = maps
      self.CommModule.Maps = maps
      self.CommModule.CompleteTx()

  def CheckIfMapsSame(self, other):
    return self.Maps.CheckIfMapsSame(other.Maps)

  def CheckForTx(self):
    self.CommModule.CheckToStartTx()

  def GetTxStatus(self):
    return self.CommModule.GetCommStatus()

  # Updates drone status:
  # Collect new data, update self/maps
  # If drone is exploring
  #   If objective has been reached objective (proximity requirement is met)
  #     If any locations of interest have been detected so far
  #       Set drone to not exploring, set objective to previous location of interest with largest "interest" value
  #     Else if no locations of interest have been detected so far
  #       Drone is still exploring, set objective to next least explored location for current drone (based on ID)
  # Else if drone is not exploring
  #   If objective has been reached objective (proximity requirement is met)
  #     Set drone to exploring, set objective to next least explored location for current drone (based on ID)
  def UpdateMaps(self):
    currMeasurement = self.DataCollect.GetNewMeasurement()
    self.UpdateLoc(currMeasurement)

    #If drone is outside search range
    if self.IsWithinSearchArea() == 0:
      self.DataCollect.publishWaypoint()
      #print "[drone] [%d]: not in exploration area" %(self.id)
      '''
      if self.x < self.bounds['xMin'] or self.x > self.bounds['xMax']:
      print "[drone] [%d]: X out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['xMin'], self.bounds['xMax'], self.x)

      if self.y < self.bounds['yMin'] or self.y > self.bounds['yMax']:
      print "[drone] [%d]: Y out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['yMin'], self.bounds['yMax'], self.y)
      
      if self.z < self.bounds['zMin'] or self.z > self.bounds['zMax']:
      print "[drone] [%d]: Z out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['zMin'], self.bounds['zMax'], self.z)
      '''
      return
      
    #currMeasurement = self.DataCollect.GetNewMeasurement()
    #self.UpdateLoc(currMeasurement)
    self.Maps.ProcessNewMeasurement(currMeasurement)
    dist = self.DistanceToObjective()
    withinProx = (dist < self.proximityReq)
    if withinProx:
      print "[drone] [%d]: within proximity %dm to target: distance to objective is (%f)" %(self.id, self.proximityReq, dist)
      if self.isExploring == True:
        currValMap = self.Maps.GetCurrentValueMapDist(self.t);
        if numpy.any(currValMap > 0):
          (zLocs,xLocs,yLocs) = numpy.nonzero(numpy.amax(currValMap) == currValMap)
          self.xObjective = xLocs[0]*self.xResolutionSize+self.bounds['xMin']
          self.yObjective = yLocs[0]*self.yResolutionSize+self.bounds['yMin']
          self.zObjective = zLocs[0]*self.zResolutionSize+self.bounds['zMin']
          obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
          [x,y,z] = self.Maps.GetCoordinateIndices( obj )
          self.isExploring = False;
          print "[drone] [%d]: reached target. Not exploring mode: moving to previously found event, new objective at (%f|%f|%f)=(%d|%d|%d)" %(self.id, self.xObjective, self.yObjective, self.zObjective, x, y, z)
          self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
        else:
          (leastExploredX, leastExploredY, leastExploredZ) = self.Maps.GetCurrentLeastExplored(self.t)
          self.xObjective = leastExploredX*self.xResolutionSize+self.bounds['xMin']
          self.yObjective = leastExploredY*self.yResolutionSize+self.bounds['yMin']
          self.zObjective = leastExploredZ*self.zResolutionSize+self.bounds['zMin']
          obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
          [x,y,z] = self.Maps.GetCoordinateIndices( obj )
          print "[drone] [%d]: reached target. No event found yet, exploring mode: moving to least explored, new objective at (%f|%f|%f)=(%d|%d|%d)" %(self.id, self.xObjective, self.yObjective, self.zObjective, x, y, z)
          self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
      else:
        (leastExploredX, leastExploredY, leastExploredZ) = self.Maps.GetCurrentLeastExplored(self.t)
        self.xObjective = leastExploredX*self.xResolutionSize+self.bounds['xMin']
        self.yObjective = leastExploredY*self.yResolutionSize+self.bounds['yMin']
        self.zObjective = leastExploredZ*self.zResolutionSize+self.bounds['zMin']
        obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
        [x,y,z] = self.Maps.GetCoordinateIndices( obj )
        self.isExploring = True
        print "[drone] [%d]: reached target. Exploring mode: moving to least explored, new objective at (%f|%f|%f)=(%d|%d|%d)" %(self.id, self.xObjective, self.yObjective, self.zObjective, x, y, z)
        self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
    #else:
      #print "Drone %d NOT within proximity %dm to target: distance to objective is (%f)" %(self.id, self.proximityReq, dist)
    self.PublishObj()

  def SetObj(self):
    self.xObjective = random.uniform(self.bounds['xMin'], self.bounds['xMax'])
    self.yObjective = random.uniform(self.bounds['yMin'], self.bounds['yMax'])
    self.zObjective = random.uniform(self.bounds['zMin'], self.bounds['zMax'])

    print "[drone] [%d] Setting new objective: [%f %f %f]" %(self.id, self.xObjective, self.yObjective, self.zObjective)
    self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)

  def PublishObj(self):
    self.DataCollect.publishWaypoint()

  def run(self):
    
    #TODO might be worthwhile to listen to takeoff ROS topic to better handle
    #     takeoff and initialization

    #wait for GPS to initialize before using it to set objectives
    self.x = -1000
    
    print "[drone] [%d] Waiting for GPS to init" %(self.id)
    
    while self.x < -180.0 or self.x > 180.0:
      GPSLoc = self.DataCollect.GPSReceiver.getGPS()
      self.x = GPSLoc['latitude']
      self.y = GPSLoc['longitude']
      self.z = GPSLoc['altitude']
      time.sleep(1)

    print "[drone] [%d] Initialized GPS [%f %f %f]" %(self.id, self.x, self.y, self.z)
    
    print "[drone] [%d] Starting run" %(self.id)
    while not self.end_thread:

      t_start = time.time()
      #get new air q sensor value and publish it
      self.gasSensor.PublishGasSensorValue()
      #TODO move comm into drone thread
      
      currMeasurement = self.DataCollect.GetNewMeasurement()
      self.UpdateLoc(currMeasurement)

      #check whether close enough to objective
      '''
      if self.IsWithinSearchArea() == 0:
        if self.x < self.bounds['xMin'] or self.x > self.bounds['xMax']:
        print "[drone] [%d]: X out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['xMin'], self.bounds['xMax'], self.x)

        if self.y < self.bounds['yMin'] or self.y > self.bounds['yMax']:
        print "[drone] [%d]: Y out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['yMin'], self.bounds['yMax'], self.y)
        
        if self.z < self.bounds['zMin'] or self.z > self.bounds['zMax']:
        print "[drone] [%d]: Z out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['zMin'], self.bounds['zMax'], self.z)
      '''

      dist = self.DistanceToObjective()
      withinProx = (dist < self.proximityReq)
      if dist < self.proximityReq:
        self.SetObj()
      
      #self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
      self.PublishObj()

      t_end = time.time()
      
      if t_end - t_start < 1.0:
        #print "[drone] [%d] Sleep: %f" %(self.id, t_end - t_start)
        time.sleep(1.0 - (t_end - t_start))
      #self.UpdateMaps()

      #handle comms
      #self.CheckForTx()
      #self.SendDataTx()

      #self.UpdateMapsForDataTx()
      

    









'''[DroneCommModule]-----------------------------------------------------------
  Handles communication between all drones, determines whether drones are in
  range to communicate
----------------------------------------------------------------------------'''
class DroneCommModule:
  def __init__(self,id,resolutions):
    self.id = id
    self.resolutions = resolutions
    self.CommReceiver = DroneCommSubscriber(self.id)
    self.CommReceiver.setSubscribe()
    self.TxInProgress = 0
    self.DronePartnerID = -1
    self.GridTxSub = DroneGridTxSubscriber(self.id, resolutions)
    self.GridTxSub.setSubscribe()
    self.GridTxPub = DroneGridTxPublisher(self.id, resolutions)
    self.Maps = None

  def CompleteTx(self):
    self.TxInProgress = 0
    self.GridTxSub.CompleteTx()

  def GetCommStatus(self):
    return self.TxInProgress

  def GetGridTxReceivedStatus(self):
    return self.GridTxSub.getTxReceived()

  def SetMaps(self, maps):
    self.Maps = maps
    
  def GetGridTxSubMaps(self):
    return self.GridTxSub.getMaps()

  def CheckToStartTx(self):
    self.CommReceiver.setSubscribe()
    self.GridTxSub.setSubscribe()
    (self.TxInProgress, self.DronePartnerID) = self.CommReceiver.getStatus()
    if self.TxInProgress == 1:
      self.GridTxSub.setTxSender(self.DronePartnerID)
      self.GridTxPub.setTxReceiver(self.DronePartnerID)
      print "[drone] [%d]: received proximity alert: drones %d + %d in communication range" %(self.id, self.id, self.DronePartnerID)

  def DataTx(self):
    self.GridTxSub.setSubscribe()
    if self.TxInProgress == 1:
      print "[drone] [%d]: sending data tx to %d" %(self.id, self.DronePartnerID)
      self.GridTxPub.publishGrids(self.Maps)
      #while not self.GridTxSub.sender == -1:
      #    print "Drone %d has not received data from partner drone %d, waiting 1s" %(self.id, self.DronePartnerID)
      #    time.sleep(1.0)        
      self.CommReceiver.resetStatus()
      #self.Maps.CombineMaps(self.GridTxSub.VMD, self.GridTxSub.STD, self.GridTxSub.MD, self.GridTxSub.VMR, self.GridTxSub.STR)

  def CheckDataReceived(self,other):
    print "Comparing Drone %d Grid Maps sent vs Drone %d Grid Sub received" %(self.id,other.id)
    if numpy.allclose(self.Maps.valueMapDist,other.GridTxSub.VMD) and numpy.allclose(self.Maps.sensedTimesDist,other.GridTxSub.STD) and numpy.allclose(self.Maps.minimumDistances, other.GridTxSub.MD) and numpy.allclose(self.Maps.valueMapRecent,other.GridTxSub.VMR) and numpy.allclose(self.Maps.sensedTimesRecent, other.GridTxSub.STR):
      print "ALL EQUAL"
    else:
     if not numpy.allclose(self.Maps.valueMapDist,other.GridTxSub.VMD):
       print "VMD not equal"
       a = numpy.argwhere(numpy.isclose(self.Maps.valueMapDist,other.GridTxSub.VMD).astype(int)==0)
       print a.shape
       for b in range(len(a)):
         i = a[b]
         #print "--(%d|%d|%d): self.VMD = %d | other.VMD = %d--" %(i[0],i[1],i[2],self.Maps.valueMapDist[i[0],i[1],i[2]],other.GridTxSub.VMD[i[0],i[1],i[2]]) 
     if not numpy.allclose(self.Maps.sensedTimesDist,other.GridTxSub.STD):
       print "STD not equal"
       a = numpy.argwhere(numpy.isclose(self.Maps.sensedTimesDist,other.GridTxSub.STD).astype(int)==0)
       print a.shape
       for b in range(len(a)):
         i = a[b]
         #print "--(%d|%d|%d): self.STD = %d | other.STD = %d--" %(i[0],i[1],i[2],self.Maps.sensedTimesDist[i[0],i[1],i[2]],other.GridTxSub.STD[i[0],i[1],i[2]])
       print "--(%d|%d|%d): self.STD = %d | other.STD = %d--" %(i[0]+1,i[1]+1,i[2]+1,self.Maps.sensedTimesDist[i[0]+1,i[1]+1,i[2]+1],other.GridTxSub.STD[i[0]+1,i[1]+1,i[2]+1])
     if not numpy.allclose(self.Maps.minimumDistances, other.GridTxSub.MD):
       print "MD not equal"
       a = numpy.argwhere(numpy.isclose(self.Maps.minimumDistances, other.GridTxSub.MD).astype(int)==0)
       print a.shape
       for b in range(len(a)):
         i = a[b]
         #print "--(%d|%d|%d): self.MD = %d | other.MD = %d--" %(i[0],i[1],i[2],self.Maps.minimumDistances[i[0],i[1],i[2]],other.GridTxSub.MD[i[0],i[1],i[2]])
     if not numpy.allclose(self.Maps.valueMapRecent,other.GridTxSub.VMR):
       print "VMR not equal"
       a = numpy.argwhere(numpy.isclose(self.Maps.valueMapRecent,other.GridTxSub.VMR).astype(int)==0)
       print a.shape
       for b in range(len(a)):
         i = a[b]
         #print "--(%d|%d|%d): self.VMR = %d | other.VMR = %d--" %(i[0],i[1],i[2],self.Maps.valueMapRecent[i[0],i[1],i[2]],other.GridTxSub.VMR[i[0],i[1],i[2]])
     if numpy.allclose(self.Maps.sensedTimesRecent, other.GridTxSub.STR):
       print "STR not equal"
       a = numpy.argwhere(numpy.isclose(self.Maps.sensedTimesRecent, other.GridTxSub.STR).astype(int)==0)
       print a.shape
       for b in range(len(a)):
         i = a[b]
         #print "--(%d|%d|%d): self.STR = %d | other.STR = %d--" %(i[0],i[1],i[2],self.Maps.sensedTimesRecent[i[0],i[1],i[2]],other.GridTxSub.STR[i[0],i[1],i[2]])

'''[DroneGridTxSubscriber]-----------------------------------------------------
  TODO desc
----------------------------------------------------------------------------'''
class DroneGridTxSubscriber:
  def __init__(self, id,resolutions):
    self.id = id
    self.resolutions = resolutions
    self.sender = -1
    self.received = 0
    self.sub = None
    self.VMD = numpy.zeros((self.resolutions['z'],self.resolutions['x'],self.resolutions['y']))
    self.STD = numpy.zeros((self.resolutions['z'],self.resolutions['x'],self.resolutions['y']))
    self.MD = numpy.zeros((self.resolutions['z'],self.resolutions['x'],self.resolutions['y']))
    self.VMR = numpy.zeros((self.resolutions['z'],self.resolutions['x'],self.resolutions['y']))
    self.STR = numpy.zeros((self.resolutions['z'],self.resolutions['x'],self.resolutions['y']))
  def setSubscribe(self):
    if self.sub == None:       #topic below
      self.sub = rospy.Subscriber("GridTx", GridData, self.TxReceive)
  def TxReceive(self,msg):
    if (msg.target == self.id) and (msg.sender == self.sender):
      print "[drone] [%d] Sub: Tx received from %d, resetting sender" %(self.id, self.sender)

      dstride0 = msg.VMD.layout.dim[0].stride
      dstride1 = msg.VMD.layout.dim[1].stride
      dstride2 = msg.VMD.layout.dim[2].stride
      offset = msg.VMD.layout.data_offset

      for i in range(self.resolutions['x']):
        for j in range(self.resolutions['y']):
          for k in range(self.resolutions['z']):
            self.VMD[i,j,k] = msg.VMD.data[offset + dstride0*i + dstride1*j + dstride2*k]
            self.STD[i,j,k] = msg.STD.data[offset + dstride0*i + dstride1*j + dstride2*k]
            self.MD[i,j,k]  = msg.MD.data[ offset + dstride0*i + dstride1*j + dstride2*k]
            self.VMR[i,j,k] = msg.VMR.data[offset + dstride0*i + dstride1*j + dstride2*k]
            self.STR[i,j,k] = msg.STR.data[offset + dstride0*i + dstride1*j + dstride2*k]
      self.received = 1

  def setTxSender(self, sender):
    if self.sender == -1:
      self.sender = sender
      print "[drone] [%d] Sub: Setting tx sender to: %d" %(self.id, self.sender)
    else:
      print "[drone] [%d] Sub: Already has tx sender %d, %d rejected" %(self.id, self.sender, sender)

  def CompleteTx(self):
    self.received = 0
    self.sender = -1

  def getMaps(self):
    return (self.VMD, self.STD, self.MD, self.VMR, self.STR)

  def getTxReceived(self):
    return self.received

class DroneGridTxPublisher:
  def __init__(self, id,resolutions):
    self.id = id
    self.resolutions = resolutions
    self.receiver = -1
    self.pub = rospy.Publisher("GridTx", GridData, queue_size=1)

  def setTxReceiver(self, receiver):
    if self.receiver == -1:
      self.receiver = receiver
      print "[drone] [%d] Pub: Setting tx receiver to: %d" %(self.id, self.receiver)
    else:
      print "[drone] [%d] Pub: Already has tx receiver %d, %d rejected" %(self.id, self.receiver, receiver)

  def publishGrids(self, maps):
    msg = GridData()
    msg.target = self.receiver
    msg.sender = self.id

    VMD = Float32MultiArray()
    VMD.layout.dim.append(MultiArrayDimension())
    VMD.layout.dim.append(MultiArrayDimension())
    VMD.layout.dim.append(MultiArrayDimension())
    VMD.layout.dim[0].label = "long"
    VMD.layout.dim[1].label = "lat"
    VMD.layout.dim[2].label = "alt"
    VMD.layout.dim[0].size = self.resolutions['x']
    VMD.layout.dim[1].size = self.resolutions['y']
    VMD.layout.dim[2].size = self.resolutions['z']
    VMD.layout.dim[0].stride = self.resolutions['x']*self.resolutions['y']
    VMD.layout.dim[1].stride = self.resolutions['y']
    VMD.layout.dim[2].stride = 1
    VMD.layout.data_offset = 0
    VMD.data = [0]*(self.resolutions['z']*self.resolutions['y']*self.resolutions['z'])

    STD = Float32MultiArray()
    STD.layout.dim.append(MultiArrayDimension())
    STD.layout.dim.append(MultiArrayDimension())
    STD.layout.dim.append(MultiArrayDimension())
    STD.layout.dim[0].label = "long"
    STD.layout.dim[1].label = "lat"
    STD.layout.dim[2].label = "alt"
    STD.layout.dim[0].size = self.resolutions['x']
    STD.layout.dim[1].size = self.resolutions['y']
    STD.layout.dim[2].size = self.resolutions['z']
    STD.layout.dim[0].stride = self.resolutions['x']*self.resolutions['y']
    STD.layout.dim[1].stride = self.resolutions['y']
    STD.layout.dim[2].stride = 1
    STD.layout.data_offset = 0
    STD.data = [0]*(self.resolutions['z']*self.resolutions['y']*self.resolutions['z'])

    MD = Float32MultiArray()
    MD.layout.dim.append(MultiArrayDimension())
    MD.layout.dim.append(MultiArrayDimension())
    MD.layout.dim.append(MultiArrayDimension())
    MD.layout.dim[0].label = "long"
    MD.layout.dim[1].label = "lat"
    MD.layout.dim[2].label = "alt"
    MD.layout.dim[0].size = self.resolutions['x']
    MD.layout.dim[1].size = self.resolutions['y']
    MD.layout.dim[2].size = self.resolutions['z']
    MD.layout.dim[0].stride = self.resolutions['x']*self.resolutions['y']
    MD.layout.dim[1].stride = self.resolutions['y']
    MD.layout.dim[2].stride = 1
    MD.layout.data_offset = 0
    MD.data = [0]*(self.resolutions['z']*self.resolutions['y']*self.resolutions['z'])

    VMR = Float32MultiArray()
    VMR.layout.dim.append(MultiArrayDimension())
    VMR.layout.dim.append(MultiArrayDimension())
    VMR.layout.dim.append(MultiArrayDimension())
    VMR.layout.dim[0].label = "long"
    VMR.layout.dim[1].label = "lat"
    VMR.layout.dim[2].label = "alt"
    VMR.layout.dim[0].size = self.resolutions['x']
    VMR.layout.dim[1].size = self.resolutions['y']
    VMR.layout.dim[2].size = self.resolutions['z']
    VMR.layout.dim[0].stride = self.resolutions['x']*self.resolutions['y']
    VMR.layout.dim[1].stride = self.resolutions['y']
    VMR.layout.dim[2].stride = 1
    VMR.layout.data_offset = 0
    VMR.data = [0]*(self.resolutions['z']*self.resolutions['y']*self.resolutions['z'])

    STR = Float32MultiArray()
    STR.layout.dim.append(MultiArrayDimension())
    STR.layout.dim.append(MultiArrayDimension())
    STR.layout.dim.append(MultiArrayDimension())
    STR.layout.dim[0].label = "long"
    STR.layout.dim[1].label = "lat"
    STR.layout.dim[2].label = "alt"
    STR.layout.dim[0].size = self.resolutions['x']
    STR.layout.dim[1].size = self.resolutions['y']
    STR.layout.dim[2].size = self.resolutions['z']
    STR.layout.dim[0].stride = self.resolutions['x']*self.resolutions['y']
    STR.layout.dim[1].stride = self.resolutions['y']
    STR.layout.dim[2].stride = 1
    STR.layout.data_offset = 0
    STR.data = [0]*(self.resolutions['z']*self.resolutions['y']*self.resolutions['z'])

    dstride0 = VMD.layout.dim[0].stride
    dstride1 = VMD.layout.dim[1].stride
    dstride2 = VMD.layout.dim[2].stride
    offset = VMD.layout.data_offset

    for i in range(self.resolutions['x']):
      for j in range(self.resolutions['y']):
        for k in range(self.resolutions['z']):
         VMD.data[offset + dstride0*i + dstride1*j + dstride2*k] = maps.valueMapDist[i,j,k]
         STD.data[offset + dstride0*i + dstride1*j + dstride2*k] = maps.sensedTimesDist[i,j,k]
         MD.data[ offset + dstride0*i + dstride1*j + dstride2*k] = maps.minimumDistances[i,j,k]
         VMR.data[offset + dstride0*i + dstride1*j + dstride2*k] = maps.valueMapRecent[i,j,k]
         STR.data[offset + dstride0*i + dstride1*j + dstride2*k] = maps.sensedTimesRecent[i,j,k]

    msg.VMD = VMD
    msg.STD = STD
    msg.MD = MD
    msg.VMR = VMR
    msg.STR = STR

    self.pub.publish(msg)
    self.receiver = -1

'''[DroneCommSubscriber]-------------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneCommSubscriber:
  def __init__(self, id):
    self.id = id
    self.dronePartner = -1
    self.sub = None
    self.status = 0
  def setSubscribe(self):
    if self.sub == None:
      self.sub = rospy.Subscriber("CommDetection", DroneComm, self.receive)
  def receive(self,msg):
    d1 = msg.d1
    d2 = msg.d2
    bw = msg.bw
    if d1 == self.id:
      self.dronePartner = d2
      self.status = 1
    elif d2 == self.id:
      self.dronePartner = d1
      self.status = 1
    print "[drone] [%d]: ack connection between [%d %d] bw: [%f]" %(self.id, d1, d2, bw)
  def resetStatus(self):
    self.status = 0
    self.dronePartner = -1
    print "[drone] [%d]: Resetting communication status"%(self.id)
  def getStatus(self):
    return (self.status,self.dronePartner)

'''[DroneGPSSubscriber]--------------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneGPSSubscriber:
  def __init__(self, id):
    self.id = id
    self.GPSReceiverName = "uav%d/mavros/global_position/global" %id                         #TODO MICHAEL: Change GPS topic to yours here
    self.longitude = -1000
    self.latitude = -1000
    self.altitude = -1000
    self.sub = None

  def setSubscribe(self):
    if self.sub == None:       #topic below
      self.sub = rospy.Subscriber(self.GPSReceiverName, NavSatFix, self.GPSReceive)         #TODO MICHAEL: Change GPS topic to yours AND/OR here

  def GPSReceive(self,msg):
    self.longitude = msg.longitude
    self.latitude = msg.latitude
    self.altitude = msg.altitude
    #print "Drone %d: received GPS location (%f|%f|%f)" %(self.id, msg.longitude, msg.latitude, msg.altitude)

  def getGPS(self):
    #print "Get location issued: location is now (%f|%f|%f)" %(self.longitude, self.latitude, self.altitude)
    return {'longitude':self.longitude,'latitude':self.latitude,'altitude':self.altitude}

'''[DroneWaypointPublisher]----------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneWaypointPublisher:
  def __init__(self, id):
    self.id = id
    self.WaypointPublisherName = "uav%d/Obj" %id                          # TODO Michael: Set your script to listen to this objective publisher
    self.longitude = 1000
    self.latitude = 1000
    self.altitude = 1000      #topic below
    self.pub = rospy.Publisher(self.WaypointPublisherName, NavSatFix, queue_size=1)

  def setWaypoint(self, waypoint):
    self.longitude = waypoint['longitude']
    self.latitude = waypoint['latitude']
    self.altitude = waypoint['altitude']
    #print "Drone %d objective publisher: objective is now (%f|%f|%f)" %(self.id, self.longitude, self.latitude, self.altitude)

  def publishWaypoint(self):
    msg = NavSatFix()
    msg.longitude = self.longitude
    msg.latitude = self.latitude
    msg.altitude = self.altitude
    #print "Drone %d: publishing objective (%f|%f|%f) on %s" %(self.id, self.longitude, self.latitude, self.altitude, self.WaypointPublisherName)
    self.pub.publish(msg)

'''[DroneSensorSubscriber]-----------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneSensorSubscriber:
  def __init__(self, id):
    self.id = id
    self.SensorReceiverName = "FakeDroneSensor_%d" %id
    self.value = 0
    self.sub = None

  def setSubscribe(self):
    if self.sub == None:
      self.sub = rospy.Subscriber(self.SensorReceiverName, GasSensorData, self.SensorReceive)

  def SensorReceive(self,msg):
    self.value = msg.value
    #print "Drone %d: received sensor data value %f" %(self.id, self.value)

  def getValue(self):
    #print "Get sensor value issued: last received value is %f" %(self.value)
    return self.value










# Drone data aggregation class. Collects sensor data of interest for drone.
# NEEDS UPDATING. CONFIGURED FOR PYTHON STANDALONE TEST RUNS
'''[DroneDataCollection]-------------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneDataCollection:
  # Constructor:
  # NEEDS UPDATING. CONFIGURED FOR PYTHON STANDALONE TEST RUNS
  # Standalone python test implementation keeps track of drone location/objective, time, maintains
  # a map of fake events which determine fake sensor data output
  def __init__(self, id):
    self.id = id
    self.GPSReceiverName = "iris_%d/mavros/state" %id
    self.GPSReceiver = DroneGPSSubscriber(id)
    self.WaypointPublisherName = "uav%d/Obj" %id
    self.WaypointPublisher = DroneWaypointPublisher(id)
    self.SensorReceiverName = "FakeDroneSensor_%d" %id
    self.SensorReceiver = DroneSensorSubscriber(id)
    self.GPSReceiver.setSubscribe()
    GPSLoc = self.GPSReceiver.getGPS()
    #while (GPSLoc['latitude'] == -1000):
    #    self.GPSReceiver.setSubscribe()
    #    GPSLoc = self.GPSReceiver.getGPS()
    self.x = GPSLoc['latitude']
    self.y = GPSLoc['longitude']
    self.z = GPSLoc['altitude']
    #print "Start GPS for drone %d dataaggregator is set! Start GPS is: (%f|%f|%f)" %(self.id, self.x, self.y, self.z)
    self.xObjective = -1000
    self.yObjective = -1000
    self.zObjective = -1000
    self.t = 0
    self.DroneComms = None

  # Should pull GPS/sensor data from NAVIO and return data in DroneMeasurement form
  def GetNewMeasurement(self):
    self.GPSReceiver.setSubscribe()
    self.SensorReceiver.setSubscribe()
    GPSLoc = self.GPSReceiver.getGPS()
    SensorVal = self.SensorReceiver.getValue()
    self.t = self.t + 1
    self.x = GPSLoc['latitude']
    self.y = GPSLoc['longitude']
    self.z = GPSLoc['altitude']
    return DroneMeasurement(self.x, self.y, self.z, self.t, SensorVal)

  # Update maintained drone current location
  def UpdateObjective(self, x, y, z):
    self.xObjective = x
    self.yObjective = y
    self.zObjective = z
    self.WaypointPublisher.setWaypoint({'latitude':self.xObjective,'longitude':self.yObjective,'altitude':self.zObjective})
    #print "Updating objective for drone %d dataaggregator: (%f|%f|%f)" %(self.id, self.yObjective, self.xObjective, self.zObjective)

  def GetGPS(self):
    return {'x':self.x,'y':self.y,'z':self.z}

  def publishWaypoint(self):
    self.WaypointPublisher.publishWaypoint()





















#TODO comment out unused things past this point
# Drone Mapping Class. Maintains the drone's ID, sniffradius, aging exponent, exploration area dimensions,
# measurements (measured values for each grid location visited)
# valueMapDist (locations of detected events of interest based on closest proximity)
# sensedTimesDist (earliest time at which location was within sniff radius based on closest proximity)
# minimumDistances (each grid location's minimum distance to drone detected during flight)
# valueMapRecent (locations of detected events of interest based on most recent update within sniff radius)
# sensedTimesRecent (earliest time at which location was within sniff radius based on most recent update within sniff radius)
# x/y/z (distance grids used to determine current locations distance to all points in the grid)
'''[DroneObjectiveMapping]-----------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneObjectiveMapping:
  # Constructor. Requires drone ID and grid dimensions as input. The
  # maintained grids are initially set to empty.
  def __init__(self, bounds, resolutions, id):
    self.bounds = bounds;
    self.resolutions = resolutions
    self.id = id
    self.agingExponent = 0.01
    self.sniffRadius = 2
    self.measurements = []

    self.valueMapDist = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))
    self.sensedTimesDist = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))
    self.minimumDistances = 100000 * numpy.ones(( resolutions['z'], resolutions['x'], resolutions['y'] ))
    self.valueMapRecent = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))
    self.sensedTimesRecent = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))

    tempX = numpy.arange(resolutions['x'])
    tempY = numpy.arange(resolutions['y'])
    tempZ = numpy.arange(resolutions['z'])
    self.x, self.y, self.z = numpy.meshgrid(tempX,tempY,tempZ)
    self.x = numpy.transpose(self.x)
    self.y = numpy.transpose(self.y)
    self.z = numpy.transpose(self.z)

  # Update drone maps based on current drone location/time:
  # Get drone data (GPS location, time, sensor value data)
  # Create a grid of distances to each grid location based on current location
  # Create a grid of "drone sight": anything outside of sniffRadius is inf distance away
  # Update all maps based on time or distance
  # If enough time has passed (based on agingExponent), reset values in minimumDistances grid to Inf
  def UpdateMaps(self, measurement):
    [x,y,z] = self.GetCoordinateIndices( measurement )
    #print "Drone is at coordinates (%d|%d|%d)" %(x,y,z)
    t = measurement.t
    val = measurement.value
    actualDistances = numpy.power(((self.x - x) ** 2) + ((self.y - y) ** 2) + ((self.z - z) ** 2), 1./2.)
    distancesInRange = actualDistances
    distGTsniffRadius = (actualDistances > self.sniffRadius).astype(int)
    distLTEQsniffRadius = (numpy.logical_not(distGTsniffRadius)).astype(int)
    distancesInRange = (distancesInRange * distLTEQsniffRadius) + (100000 * distGTsniffRadius)

    GTPrevMinDist = (distancesInRange > self.minimumDistances).astype(int)
    LTEQPrevMinDist = (numpy.logical_not(GTPrevMinDist)).astype(int)
    self.valueMapDist = self.valueMapDist * GTPrevMinDist + LTEQPrevMinDist * val
    self.sensedTimesDist = self.sensedTimesDist * GTPrevMinDist + LTEQPrevMinDist * t
    self.minimumDistances = self.minimumDistances * GTPrevMinDist + LTEQPrevMinDist * actualDistances
    self.valueMapRecent = self.valueMapRecent * distGTsniffRadius + distLTEQsniffRadius * val
    self.sensedTimesRecent = self.sensedTimesRecent * distGTsniffRadius + distLTEQsniffRadius * t

    # If non-0 values have been recorded, reset values that are too old
    if numpy.size(self.valueMapDist[ numpy.where( self.valueMapDist > 1 ) ]) > 0:
      ind = (numpy.absolute(self.sensedTimesDist - t) * self.agingExponent) > 1
      self.minimumDistances = (self.minimumDistances * numpy.logical_not(ind)) + (ind * 10000)

  # Returns the current valueMapDist after its been aged based on the agingExponent
  def GetCurrentValueMapDist(self, t):
    map = self.valueMapDist * numpy.exp((self.agingExponent*-1) * (t - self.sensedTimesDist))
    return map

  # Returns the current valueMapRecent after its been aged based on the agingExponent
  def GetCurrentValueMapRecent(self, t):
    map = self.valueMapRecent * numpy.exp((self.agingExponent*-1) * (t - self.sensedTimesRecent))
    return map

  # Returns the least explored locations based on minimumDistances. Sends different drones to different areas
  # based on minimumDistances grid
  def GetCurrentLeastExplored(self,t):
    distances = numpy.sort(self.minimumDistances, axis=None)[::-1]
    (leastExploredZ, leastExploredX, leastExploredY) = numpy.where((self.minimumDistances == distances[5*(self.id-1)]).astype(int) > 0)
    print "Least Explored value is %f, least explored loc is at (%d|%d|%d) with least explored value %f" %(distances[0], leastExploredX[0], leastExploredY[0], leastExploredZ[0], self.minimumDistances[leastExploredZ[0]][leastExploredX[0]][leastExploredY[0]])
    return (leastExploredX[0], leastExploredY[0], leastExploredZ[0])

  # Updates the measurements list. If the new measurement is close to a previous measurement, update the previous measurement.
  # Return the number of previous measurements that are close to the current measurement.
  def ReplaceSameLocation( self, measurement ):
    num = 0
    for (ind, data) in enumerate( self.measurements ):
      if self.measurements[ind].isClose( measurement ):
        self.measurements[ind].t = measurement.t
        self.measurements[ind].value = measurement.value
        num = num + 1
    return num

  # Update the maps based on the input measurement. If the new measurement is not seen before, append it to measurements. Otherwise
  # update the current measurements list only.
  def ProcessNewMeasurement( self, measurement ):
    self.UpdateMaps( measurement )
    if self.ReplaceSameLocation( measurement ) == 0:
      self.measurements.append(measurement)

  # Given an input measurement, return the respective grid coordinate
  def GetCoordinateIndices( self, measurement ):
    xIndex = math.ceil(( measurement.x - self.bounds['xMin'] ) / ( self.bounds['xMax'] - self.bounds['xMin'] ) * self.resolutions['x'])
    yIndex = math.ceil(( measurement.y - self.bounds['yMin'] ) / ( self.bounds['yMax'] - self.bounds['yMin'] ) * self.resolutions['y'])
    zIndex = math.ceil(( measurement.z - self.bounds['zMin'] ) / ( self.bounds['zMax'] - self.bounds['zMin'] ) * self.resolutions['z'])
    return [xIndex, yIndex, zIndex]
  
  # Sets current DroneObjectiveMapping object's valueMapDist, sensedTimesDist, valueMapRecent, sensedTimesRecent, and minimumDistances 
  # to that of DroneObjectiveMapping object "other"
  def Merge(self, other):
    self.valueMapDist = other.valueMapDist
    self.sensedTimesDist = other.sensedTimesDist
    self.minimumDistances = other.minimumDistances
    self.valueMapRecent = other.valueMapRecent
    self.sensedTimesRecent = other.sensedTimesRecent
  

  def CheckIfMapsSame(self, other):
    if numpy.allclose(self.valueMapDist,other.valueMapDist) and numpy.allclose(self.sensedTimesDist,other.sensedTimesDist) and numpy.allclose(self.minimumDistances, other.minimumDistances) and numpy.allclose(self.valueMapRecent,other.valueMapRecent) and numpy.allclose(self.sensedTimesRecent, other.sensedTimesRecent):
      print "ALL EQUAL"
      return 1
    else:
      if not numpy.allclose(self.valueMapDist,other.valueMapDist):
        print "VMD not equal"
        a = numpy.argwhere(numpy.isclose(self.valueMapDist,other.valueMapDist).astype(int)==0)
        print a.shape
        for b in range(len(a)):
          i = a[b]
          #print "--(%d|%d|%d): self.VMD = %d | other.VMD = %d--" %(i[0],i[1],i[2],self.valueMapDist[i[0],i[1],i[2]],other.valueMapDist[i[0],i[1],i[2]]) 
      if not numpy.allclose(self.sensedTimesDist,other.sensedTimesDist):
        print "STD not equal"
        a = numpy.argwhere(numpy.isclose(self.sensedTimesDist,other.sensedTimesDist).astype(int)==0)
        print a.shape
        for b in range(len(a)):
          i = a[b]
          #print "--(%d|%d|%d): self.STD = %d | other.STD = %d--" %(i[0],i[1],i[2],self.sensedTimesDist[i[0],i[1],i[2]],other.sensedTimesDist[i[0],i[1],i[2]])
      if not numpy.allclose(self.minimumDistances, other.minimumDistances):
        print "MD not equal"
        a = numpy.argwhere(numpy.isclose(self.minimumDistances,other.minimumDistances).astype(int)==0)
        print a.shape
        for b in range(len(a)):
          i = a[b]
          #print "--(%d|%d|%d): self.MD = %d | other.MD = %d--" %(i[0],i[1],i[2],self.minimumDistances[i[0],i[1],i[2]],other.minimumDistances[i[0],i[1],i[2]])
      if not numpy.allclose(self.valueMapRecent,other.valueMapRecent):
        print "VMR not equal"
        a = numpy.argwhere(numpy.isclose(self.valueMapRecent,other.valueMapRecent).astype(int)==0)
        print a.shape
        for b in range(len(a)):
          i = a[b]
          #print "--(%d|%d|%d): self.VMR = %d | other.VMR = %d--" %(i[0],i[1],i[2],self.valueMapRecent[i[0],i[1],i[2]],other.valueMapRecent[i[0],i[1],i[2]])
      if not numpy.allclose(self.sensedTimesRecent,other.sensedTimesRecent):
        print "STR not equal"
        a = numpy.argwhere(numpy.isclose(self.sensedTimesRecent,other.sensedTimesRecent).astype(int)==0)
        print a.shape
        for b in range(len(a)):
          i = a[b]
          #print "--(%d|%d|%d): self.STR = %d | other.STR = %d--" %(i[0],i[1],i[2],self.sensedTimesRecent[i[0],i[1],i[2]],other.sensedTimesRecent[i[0],i[1],i[2]])
      return 0

  # Sets current DroneObjectiveMapping object's valueMapDist, sensedTimesDist, valueMapRecent, sensedTimesRecent, and minimumDistances 
  # to that of DroneObjectiveMapping object "other"
  def MergeMaps(self, VMD, STD, MD, VMR, STR):
    self.valueMapDist = VMD
    self.sensedTimesDist = STD
    self.minimumDistances = MD
    self.valueMapRecent = VMR
    self.sensedTimesRecent = STR

  # Combines current DroneObjectiveMapping object with that of "other" into a new returned object.
  # For distance based grids, saves the values of the drone which has the lower minimumDistance value per grid location (closest visit)
  # For time based grids, saves the values of the drone which has the lower sesnedTimesRecent value per grid location (most recent visit)
  def Combine(self, other):
    combined = DroneObjectiveMapping( self.bounds, self.resolutions, self.id )

    distSelfSmaller = (self.minimumDistances < other.minimumDistances).astype(int)
    distOtherSmaller = (self.minimumDistances > other.minimumDistances).astype(int)
    distEqual = numpy.ones(distSelfSmaller.shape)-distSelfSmaller-distOtherSmaller

    tempSelf = self.valueMapDist * distSelfSmaller
    tempOther = other.valueMapDist * distOtherSmaller
    tempEqual = numpy.maximum((self.valueMapDist * distEqual),(other.valueMapDist * distEqual))
    combined.valueMapDist = tempSelf + tempOther + tempEqual

    tempSelf = self.sensedTimesDist * distSelfSmaller
    tempOther = other.sensedTimesDist * distOtherSmaller
    tempEqual = numpy.maximum((self.sensedTimesDist * distEqual),(other.sensedTimesDist * distEqual))
    combined.sensedTimesDist = tempSelf + tempOther + tempEqual

    combined.minimumDistances = numpy.minimum(self.minimumDistances,other.minimumDistances)

    selfMoreRecent = (self.sensedTimesRecent > other.sensedTimesRecent).astype(int)
    otherMoreRecent = (other.sensedTimesRecent < STR).astype(int)
    bothMoreRecent = numpy.ones(otherMoreRecent.shape)-selfMoreRecent-otherMoreRecent

    tempSelf = self.valueMapRecent * selfMoreRecent
    tempOther = other.valueMapRecent * otherMoreRecent
    tempEqual = numpy.maximum((self.valueMapRecent * bothMoreRecent),(other.valueMapRecent * bothMoreRecent))
    combined.valueMapRecent = tempSelf + tempOther + tempEqual

    combined.sensedTimesRecent = numpy.maximum(self.sensedTimesRecent,other.sensedTimesRecent)

    return combined

  # Combines current DroneObjectiveMapping object with that of "other" into a new returned object.
  # For distance based grids, saves the values of the drone which has the lower minimumDistance value per grid location (closest visit)
  # For time based grids, saves the values of the drone which has the lower sesnedTimesRecent value per grid location (most recent visit)
  def CombineReceivedMaps(self, commModule):
    combined = DroneObjectiveMapping( self.bounds, self.resolutions, self.id )

    (VMD, STD, MD, VMR, STR) = commModule.GetGridTxSubMaps()

    distSelfSmaller = (self.minimumDistances < MD).astype(int)
    distOtherSmaller = (self.minimumDistances > MD).astype(int)
    distEqual = numpy.ones(distSelfSmaller.shape)-distSelfSmaller-distOtherSmaller

    tempSelf = self.valueMapDist * distSelfSmaller
    tempOther = VMD * distOtherSmaller
    tempEqual = numpy.maximum((self.valueMapDist * distEqual),(VMD * distEqual))
    combined.valueMapDist = tempSelf + tempOther + tempEqual

    tempSelf = self.sensedTimesDist * distSelfSmaller
    tempOther = STD * distOtherSmaller
    tempEqual = numpy.maximum((self.sensedTimesDist * distEqual),(STD * distEqual))
    combined.sensedTimesDist = tempSelf + tempOther + tempEqual

    combined.minimumDistances = numpy.minimum(self.minimumDistances,MD)

    selfMoreRecent = (self.sensedTimesRecent > STR).astype(int)
    otherMoreRecent = (self.sensedTimesRecent < STR).astype(int)
    bothMoreRecent = numpy.ones(otherMoreRecent.shape)-selfMoreRecent-otherMoreRecent

    tempSelf = self.valueMapRecent * selfMoreRecent
    tempOther = VMR * otherMoreRecent
    tempEqual = numpy.maximum((self.valueMapRecent * bothMoreRecent),(VMR * bothMoreRecent))
    combined.valueMapRecent = tempSelf + tempOther + tempEqual

    combined.sensedTimesRecent = numpy.maximum(self.sensedTimesRecent,STR)

    return combined

# Currently unused, may be used later for drone-to-drone communication
'''[DroneCommunicator]---------------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneCommunicator:
  def __init__(self):
    self.ipAddress = '';
    self.port = 5005;
    self.bufferSize = 1024;

    try:
      self.socketHandle = socket.socket( socket.AF_INET, socket.SOCK_STREAM );
      self.socketHandle.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )
    except socket.error as msg:
      s = None;

    self.StartServer();
    self.WaitForDroneConnection();

  def StartServer(self):
    try:
      self.socketHandle.bind( ( self.ipAddress, self.port ) );
      self.socketHandle.listen( 1 );
    except socket.error as msg:
      self.socketHandle.close();
      self.socketHandle = None;

    if self.socketHandle is None:
      print 'Server Socket Creation Failed';

  def WaitForDroneConnection(self):
    self.connectionHandle, self.droneAddress = self.socketHandle.accept();
    print 'Connected by ', self.connectionHandle

  def ReceiveNewData(self):
    data = self.connectionHandle.recv( self.bufferSize );
    return data;

  def SendNewData(self, data):
    self.connectionHandle.send( data );

# Holder class for measurements, maintains GPS location, time, and value variables
'''[DroneMeasurement]----------------------------------------------------------
  TODO
----------------------------------------------------------------------------'''
class DroneMeasurement:
  def __init__( self, x, y, z, tTime, value ):
    self.x = x;
    self.y = y;
    self.z = z;
    self.t = tTime;
    self.value = value;
    self.proximityReq = 1

  # Returns whether input measurement is close to current measurement (within proximity requirement)
  def isClose(self,measurement):
    if ( ( self.x - measurement.x ) ** 2 + ( self.y - measurement.y ) ** 2 + ( self.z - measurement.z )** 2) ** (1./2) <= self.proximityReq:
      return True;
    else:
      return False;

  # Prints measurement to JSON format
  def toJSON(self):
    return json.dumps( self, default=lambda o: o.__dict__ );
