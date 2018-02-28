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
from drone.msg import GridTx
from sensor_msgs.msg import NavSatFix

# Drone Class. Maintains ID, exploration flag, GPS location, time, objective GPS location, exploration grid bounds, 
# resolution of exploration grid, data aggregator object, exploration mapping object.
class Drone:
    # Constructor. Requires start GPS location, exploration area size (in feet), grid resolution, and id
    def __init__(self, xRange, yRange, zRange, resolutions, id):
        self.id = id
        self.moduleName = "Drone_%d" %id
        #rospy.init_node(self.moduleName, anonymous = True)
        self.proximityReq = 1                       # Max distance to target to consider objective location reached
        self.isExploring = True
        self.id = id
        self.t = 0

        self.DataCollect = DroneDataCollection(id)
        startGPS = self.DataCollect.GetGPS()

        self.x = startGPS['x']
        self.y = startGPS['y']
        self.z = startGPS['z']

        # Calculate explorable area grid bounds in terms of GPS locations based on input exploration area size
        self.xMax = xRange['max']
        self.xMin = xRange['min']
        self.yMax = yRange['max']
        self.yMin = yRange['min']
        self.zMax = zRange['max']
        self.zMin = zRange['min']
        self.bounds = {'xMax':self.xMax,'xMin':self.xMin,'yMax':self.yMax,'yMin':self.yMin,'zMin':self.zMin,'zMax':self.zMax}
        self.resolutions = resolutions
        self.xResolutionSize = (self.bounds['xMax'] - self.bounds['xMin'])/self.resolutions['x']
        self.yResolutionSize = (self.bounds['yMax'] - self.bounds['yMin'])/self.resolutions['y']
        self.zResolutionSize = (self.bounds['zMax'] - self.bounds['zMin'])/self.resolutions['z']
        self.Maps = DroneObjectiveMapping( self.bounds, resolutions, id )

        # If drone is starting within search area, set initial start destination to furthest exploration grid corner
        # If drone is not starting within search area, set objective to closest grid location
        # TODO: Better start destination
        #print "Drone %d sx: %f/%f/%f" %(self.x, self.xMin, self.xMax)
        #print "Drone %d y: %f/%f/%f" %(self.y, self.yMin, self.yMax)
        #print "Drone %d z: %f/%f/%f" %(self.z, self.zMin, self.zMax)
        if not ((self.x <= self.xMax) and (self.x >= self.xMin)):
            if (self.x > self.xMax):
                self.xObjective = self.xMax
            else:
                self.xObjective = self.xMin
        else:
            if self.x < ((bounds['xMax'] - bounds['xMin']) / 2):
                self.xObjective = bounds['xMax']
            else:
                self.xObjective = bounds['xMin']
        if not ((self.y <= self.yMax) and (self.y >= self.yMin)):
            if (self.y > self.yMax):
                self.yObjective = self.yMax
            else:
                self.yObjective = self.yMin
        else:
            if self.y < ((bounds['yMax'] - bounds['yMin']) / 2):
                self.yObjective = bounds['yMax']
            else:
                self.yObjective = bounds['yMin']
        if not ((self.z <= self.zMax) and (self.z >= self.zMin)):
            if (self.z > self.zMax):
                self.zObjective = self.zMax
            else:
                self.zObjective = self.zMin
        else:
            if self.z < ((bounds['zMax'] - bounds['zMin']) / 2):
                self.zObjective = bounds['zMax']
            else:
                self.zObjective = bounds['zMin']
        self.PublishObj()

        self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
        obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
        [x,y,z] = self.Maps.GetCoordinateIndices(obj)
        rospy.loginfo("Drone %d: Setting start objective (%f|%f|%f) = (%d|%d|%d)" %(self.id, self.yObjective, self.xObjective, self.zObjective,x,y,z))
        self.CommModule = DroneCommModule(id)
        self.DataCollect.SetCommModule(self.CommModule)
        self.CommModule.SetDataCollect(self.DataCollect)

    # Update drone GPS location based on input measurement
    def UpdateLoc(self, measurement):
        oldM = DroneMeasurement(self.x, self.y, self.z, 0, 0)
        [x0,y0,z0] = self.Maps.GetCoordinateIndices( oldM )
        #print "Drone %d: PREMOVE: GPS:(%f|%f|%f)=(%d, %d, %d)"%(self.id, self.x, self.y,  self.z, x, y, z)
        self.x = measurement.x
        self.y = measurement.y
        self.z = measurement.z
        self.t = measurement.t
        newM = DroneMeasurement(self.x, self.y, self.z, 0, 0)
        [x,y,z] = self.Maps.GetCoordinateIndices( newM )
        #print "Drone %d: POSTMOVE: GPS:(%f|%f|%f)=(%d, %d, %d)"%(self.id, self.x, self.y,  self.z, x, y, z)
        obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
        [xObj,yObj,zObj] = self.Maps.GetCoordinateIndices( obj )
        if (x == x0) and (y == y0) and (z == z0):
            outstr = "Drone %d: COORDINATES DID NOT CHANGE: GPS:(%f|%f|%f)=(%d, %d, %d), Obj:(%f|%f|%f)=(%d, %d, %d)" %(self.id, self.x, self.y,  self.z, x, y, z, self.xObjective, self.yObjective, self.zObjective, xObj, yObj, zObj)
        else:
            # Print current location, exploration status, and objective
            if self.isExploring is True:
                outstr = "Drone %d: Exploring, GPS:(%f|%f|%f)=(%d, %d, %d), Obj:(%f|%f|%f)=(%d, %d, %d)" %(self.id, self.x, self.y,  self.z, x, y, z, self.xObjective, self.yObjective, self.zObjective, xObj, yObj, zObj)
            else:
                outstr = "Drone %d: NOT Exploring, GPS:(%f|%f|%f)=(%d, %d, %d), Obj:(%f|%f|%f)=(%d, %d, %d)" %(self.id, self.x, self.y,  self.z, x, y, z, self.xObjective, self.yObjective, self.zObjective, xObj, yObj, zObj)
        rospy.loginfo(outstr)

    # Calculates distance to objective (in meters based on GPS coordinates)
    def DistanceToObjective(self):
        xDist = (self.x - self.xObjective)*111111.11
        yDist = (self.y - self.yObjective)*111111.11
        zDist = (self.z - self.zObjective)
        return ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2)

    def IsWithinSearchArea(self):
        if ((self.x <= self.xMax) and (self.x >= self.xMin) and (self.y <= self.yMax) and (self.y >= self.yMin) and (self.z <= self.zMax) and (self.z >= self.zMin)):
            return 1
        else:
            return 0

    # Updates drone status:
    # Collect new data, update self/maps
    # If drone is exploring
    #   If objective has been reached objective (proximity requirement is met)
    #       If any locations of interest have been detected so far
    #           Set drone to not exploring, set objective to previous location of interest with largest "interest" value
    #       Else if no locations of interest have been detected so far
    #           Drone is still exploring, set objective to next least explored location for current drone (based on ID)
    # Else if drone is not exploring
    #   If objective has been reached objective (proximity requirement is met)
    #       Set drone to exploring, set objective to next least explored location for current drone (based on ID)
    def UpdateStatus(self):
        currMeasurement = self.DataCollect.GetNewMeasurement()
        self.UpdateLoc(currMeasurement)

        #If drone is outside search range
        if self.IsWithinSearchArea() == 0:
            self.DataCollect.WaypointPublisher.publishWaypoint()
            rospy.loginfo("Drone %d is not in exploration area [X:(%f|%f)/Y:(%f|%f)/Z:(%f|%f)] - current location is (%f|%f|%f)" %(self.id, self.xMin, self.xMax, self.yMin, self.yMax, self.zMin, self.zMax, self.x, self.y, self.z))
            return

        #TODO: Is this correct if drone is currently swapping map information with another drone
        if self.CommModule.Update() == 1:
            self.CommModule.DataTx()
            
        #currMeasurement = self.DataCollect.GetNewMeasurement()
        #self.UpdateLoc(currMeasurement)
        self.Maps.ProcessNewMeasurement(currMeasurement)
        dist = self.DistanceToObjective()
        withinProx = (dist < self.proximityReq)
        if withinProx:
            outstr =  "Drone %d is within proximity %dm to target: distance to objective is (%f)" %(self.id, self.proximityReq, dist)
            rospy.loginfo(outstr)
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
                    outstr =  "Drone %d reached target. Not exploring mode: moving to previously found event, new objective at (%f|%f|%f)=(%d|%d|%d)" %(self.id, self.xObjective, self.yObjective, self.zObjective, x, y, z)
                    rospy.loginfo(outstr)
                    self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
                else:
                    (leastExploredX, leastExploredY, leastExploredZ) = self.Maps.GetCurrentLeastExplored(self.t)
                    self.xObjective = leastExploredX*self.xResolutionSize+self.bounds['xMin']
                    self.yObjective = leastExploredY*self.yResolutionSize+self.bounds['yMin']
                    self.zObjective = leastExploredZ*self.zResolutionSize+self.bounds['zMin']
                    obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
                    [x,y,z] = self.Maps.GetCoordinateIndices( obj )
                    outstr = "Drone %d reached target. No event found yet, exploring mode: moving to least explored, new objective at (%f|%f|%f)=(%d|%d|%d)" %(self.id, self.xObjective, self.yObjective, self.zObjective, x, y, z)
                    rospy.loginfo(outstr)
                    self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
            else:
                (leastExploredX, leastExploredY, leastExploredZ) = self.Maps.GetCurrentLeastExplored(self.t)
                self.xObjective = leastExploredX*self.xResolutionSize+self.bounds['xMin']
                self.yObjective = leastExploredY*self.yResolutionSize+self.bounds['yMin']
                self.zObjective = leastExploredZ*self.zResolutionSize+self.bounds['zMin']
                obj = DroneMeasurement(self.xObjective, self.yObjective, self.zObjective, 0, 0)
                [x,y,z] = self.Maps.GetCoordinateIndices( obj )
                self.isExploring = True
                outstr = "Drone %d reached target. Exploring mode: moving to least explored, new objective at (%f|%f|%f)=(%d|%d|%d)" %(self.id, self.xObjective, self.yObjective, self.zObjective, x, y, z)
                rospy.loginfo(outstr)
                self.DataCollect.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
        #else:
            #outstr =  "Drone %d NOT within proximity %dm to target: distance to objective is (%f)" %(self.id, self.proximityReq, dist)
            #rospy.loginfo(outstr)
        self.PublishObj()

    def PublishObj(self):
        self.DataCollect.WaypointPublisher.publishWaypoint()

class DroneCommModule:
    def __init__(self,id):
        self.id = id
        self.CommReceiver = DroneCommSubscriber(self.id)
        self.CommReceiver.setSubscribe()
        self.TxInProgress = 0
        self.DronePartnerID = -1
        self.GridTxSub = DroneGridTxSubscriber(self.id)
        self.GridTxSub.setSubscribe()
        self.GridTxPub = DroneGridTxPublisher(self.id)
        self.DataCollect = None
    def GetCommStatus(self):
        return self.TxInProgress
    def SetDataCollect(self, DC):
        self.DataCollect = DC
    def Update(self):
        self.CommReceiver.setSubscribe()
        (self.TxInProgress, self.DronePartnerID) = self.CommReceiver.getStatus()
        return self.TxInProgress
    def DataTx(self):
        if self.TxInProgress == 1:
            rospy.loginfo("Drone %d received proximity alert: drones %d + %d in communication range - doing nothing" %(self.id, self.id, self.DronePartnerID))
            self.GridTxSub.setTxSender = self.DronePartnerID
            self.GridTxPub.setTxPublisher = self.DronePartnerID
            #valueMapDist = self.DataCollect.valueMapDist
            #sensedTimesDist = self.DataCollect.sensedTimesDist
            #minimumDistances = self.DataCollect.minimumDistances
            self.CommReceiver.resetStatus()

class DroneGridTxSubscriber:
    def __init__(self, id):
        self.id = id
        self.sender = -1
        self.sub = None
    def setSubscribe(self):
        if self.sub == None:         #topic below
            self.sub = rospy.Subscriber("GridTx", GridTx, self.TxReceive)
    def TxReceive(self,msg):
        if (msg.target == self.id) and (msg.sender == self.sender):
            rospy.loginfo("Drone %d Sub: Tx received from %d, data is %d, resetting sender" %(self.id, msg.data, self.sender))
            self.sender = -1
    def setTxSender(self, sender):
        if self.sender == -1:
            self.sender = sender
            rospy.loginfo("Drone %d Sub: Setting tx sender to: %d" %(self.id, self.sender))
        else:
            rospy.loginfo("Drone %d Sub: Already has tx sender %d, %d rejected" %(self.id, self.sender, sender))

class DroneGridTxPublisher:
    def __init__(self, id):
        self.id = id
        self.receiver = -1
        self.pub = rospy.Publisher("GridTx", GridTx, queue_size=1)
    def setTxReceiver(self, receiver):
        if self.receiver == -1:
            self.receiver = receiver
            rospy.loginfo("Drone %d Pub: Setting tx receiver to: %d" %(self.id, self.receiver))
        else:
            rospy.loginfo("Drone %d Pub: Already has tx receiver %d, %d rejected" %(self.id, self.receiver, receiver))
    def publishGrid(self):
        msg = GridTx()
        msg.sender = self.longitude
        msg.receiver = self.latitude
        msg.data = 0
        #rospy.loginfo("Drone %d: publishing objective (%f|%f|%f) on %s" %(self.id, self.longitude, self.latitude, self.altitude, self.WaypointPublisherName))
        self.pub.publish(msg)

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
        if d1 == self.id:
            self.dronePartner = d2
            self.status = 1
        elif d2 == self.id:
            self.dronePartner = d1
            self.status = 1
        rospy.loginfo("Drone %d: pair (%d/%d) in communication range!" %(self.id, d1, d2))
    def resetStatus(self):
        self.status = 0
        self.dronePartner = -1
        rospy.loginfo("Drone %d: Resetting communication status"%(self.id)) 
    def getStatus(self):
        return (self.status,self.dronePartner)

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
        #rospy.loginfo("Drone %d: received GPS location (%f|%f|%f)" %(self.id, msg.longitude, msg.latitude, msg.altitude))
    def getGPS(self):
        #rospy.loginfo("Get location issued: location is now (%f|%f|%f)" %(self.longitude, self.latitude, self.altitude))
        return {'longitude':self.longitude,'latitude':self.latitude,'altitude':self.altitude}

class DroneWaypointPublisher:
    def __init__(self, id):
        self.id = id
        self.WaypointPublisherName = "FakeDroneObj_%d" %id
        self.longitude = 1000
        self.latitude = 1000
        self.altitude = 1000        #topic below
        self.pub = rospy.Publisher(self.WaypointPublisherName, NavSatFix, queue_size=1)
    def setWaypoint(self, waypoint):
        self.longitude = waypoint['longitude']
        self.latitude = waypoint['latitude']
        self.altitude = waypoint['altitude']
        #rospy.loginfo("Drone %d objective publisher: objective is now (%f|%f|%f)" %(self.id, self.longitude, self.latitude, self.altitude))
    def publishWaypoint(self):
        msg = NavSatFix()
        msg.longitude = self.longitude
        msg.latitude = self.latitude
        msg.altitude = self.altitude
        #rospy.loginfo("Drone %d: publishing objective (%f|%f|%f) on %s" %(self.id, self.longitude, self.latitude, self.altitude, self.WaypointPublisherName))
        self.pub.publish(msg)

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
        #rospy.loginfo("Drone %d: received sensor data value %f" %(self.id, self.value))
    def getValue(self):
        #rospy.loginfo("Get sensor value issued: last received value is %f" %(self.value))
        return self.value

# Drone data aggregation class. Collects sensor data of interest for drone.
# NEEDS UPDATING. CONFIGURED FOR PYTHON STANDALONE TEST RUNS
class DroneDataCollection:
    # Constructor:
    # NEEDS UPDATING. CONFIGURED FOR PYTHON STANDALONE TEST RUNS
    # Standalone python test implementation keeps track of drone location/objective, time, maintains
    # a map of fake events which determine fake sensor data output
    def __init__(self, id):
        self.id = id
        self.GPSReceiverName = "uav%d/mavros/state" %id
        self.GPSReceiver = DroneGPSSubscriber(id)
        self.WaypointPublisherName = "FakeDroneObj_%d" %id
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
        #rospy.loginfo("Start GPS for drone %d dataaggregator is set! Start GPS is: (%f|%f|%f)" %(self.id, self.x, self.y, self.z))

        self.xObjective = -1000
        self.yObjective = -1000
        self.zObjective = -1000
        self.t = 0
        self.DroneComms = None

    def SetCommModule(self, DC):
        self.DroneComms = DC

    # Should pull GPS/sensor data from NAVIO and return data in DroneMeasurement form
    # NEEDS UPDATING. Current standalone python test implementation performs a fake move and calculates fake values
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
        #rospy.loginfo("Updating objective for drone %d dataaggregator: (%f|%f|%f)" %(self.id, self.yObjective, self.xObjective, self.zObjective))

    def GetGPS(self):
        return {'x':self.x,'y':self.y,'z':self.z}

# Drone Mapping Class. Maintains the drone's ID, sniffradius, aging exponent, exploration area dimensions,
#   measurements (measured values for each grid location visited)
#   valueMapDist (locations of detected events of interest based on closest proximity)
#   sensedTimesDist (earliest time at which location was within sniff radius based on closest proximity)
#   minimumDistances (each grid location's minimum distance to drone detected during flight)
#   valueMapRecent (locations of detected events of interest based on most recent update within sniff radius)
#   sensedTimesRecent (earliest time at which location was within sniff radius based on most recent update within sniff radius)
#   x/y/z (distance grids used to determine current locations distance to all points in the grid)
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
        #outstr = "Drone is at coordinates (%d|%d|%d)" %(x,y,z)
        #rospy.loginfo(outstr)
        t = measurement.t
        val = measurement.value
        actualDistances = numpy.power(((self.x - x) ** 2) + ((self.y - y) ** 2) + ((self.z - z) ** 2), 1./2.)
        distances = actualDistances
        distGTsniffRadius = (distances > self.sniffRadius).astype(int)
        distLTEQsniffRadius = (numpy.logical_not(distGTsniffRadius)).astype(int)
        distances = (distances * distLTEQsniffRadius) + (10000 * distGTsniffRadius)

        actualGTminimum = (actualDistances > self.minimumDistances).astype(int)
        actualLTEQminimum = (numpy.logical_not(actualGTminimum)).astype(int)
        self.valueMapDist = self.valueMapDist * actualGTminimum + actualLTEQminimum * val
        self.sensedTimesDist = self.sensedTimesDist * actualGTminimum + actualLTEQminimum * t
        self.minimumDistances = self.minimumDistances * actualGTminimum + actualLTEQminimum * actualDistances
        #print(self.valueMapDist)
        #print(self.sensedTimesDist)
        #print(self.minimumDistances)

        self.valueMapRecent = self.valueMapRecent * distGTsniffRadius + distLTEQsniffRadius * val
        self.sensedTimesRecent = self.sensedTimesRecent * distGTsniffRadius + distLTEQsniffRadius * t
        #print(self.valueMapDist)
        #print(self.sensedTimesDist)
        #print(self.minimumDistances)

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

    # Combines current DroneObjectiveMapping object with that of "other" into a new returned object.
    # For distance based grids, saves the values of the drone which has the lower minimumDistance value per grid location (closest visit)
    # For time based grids, saves the values of the drone which has the lower sesnedTimesRecent value per grid location (most recent visit)
    def Combine(self, other):
        combined = DroneObjectiveMapping( self.bounds, self.resolutions, id )
        combined.eventMap = self.eventMap

        smallerDist = (self.minimumDistances < other.minimumDistances).astype(int)
        smallerDistInv = (numpy.logical_not(smallerDist)).astype(int)
        tempSelf = self.valueMapDist * smallerDist
        tempOther = other.valueMapDist * smallerDistInv
        combined.valueMapDist = tempSelf + tempOther
        tempSelf = self.sensedTimesDist * smallerDist
        tempOther = other.sensedTimesDist * smallerDistInv
        combined.sensedTimesDist = tempSelf + tempOther
        #numpy.savetxt("minDist1.txt",self.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist2.txt",other.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1<minDist2.txt",smallerDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1>=minDist2.txt",smallerDistInv,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1.txt",self.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2.txt",other.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1*(minDist1<minDist2).txt",tempSelf,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2*(minDist1>=minDist2).txt",tempOther,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("combined_sensedTimes.txt",combined.sensedTimesDist,fmt='%1.3f',delimiter=",")
        tempSelf = self.minimumDistances * smallerDist
        tempOther = other.minimumDistances * smallerDistInv
        combined.minimumDistances = tempSelf + tempOther
        #numpy.savetxt("combined_minDist.txt",combined.minimumDistances,fmt='%1.3f',delimiter=",")

        moreRecentSensedTimes = (self.sensedTimesRecent >= other.sensedTimesRecent).astype(int)
        moreRecentSensedTimesInv = (numpy.logical_not(moreRecentSensedTimes)).astype(int)
        tempSelf = self.valueMapRecent * moreRecentSensedTimes
        tempOther = other.valueMapRecent * moreRecentSensedTimesInv
        combined.valueMapRecent = tempSelf + tempOther
        tempSelf = self.sensedTimesRecent * moreRecentSensedTimes
        tempOther = other.sensedTimesRecent * moreRecentSensedTimesInv
        combined.sensedTimesDist = tempSelf + tempOther
        #numpy.savetxt("minDist1.txt",self.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist2.txt",other.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1<minDist2.txt",smallerDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1>=minDist2.txt",smallerDistInv,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1.txt",self.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2.txt",other.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1*(minDist1<minDist2).txt",tempSelf,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2*(minDist1>=minDist2).txt",tempOther,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("combined_sensedTimes.txt",combined.sensedTimesDist,fmt='%1.3f',delimiter=",")
    
        return combined

# Currently unused, may be used later for drone-to-drone communication
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
