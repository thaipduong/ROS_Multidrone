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
import numpy as np
import random
import time
import json
import threading
#from drone.msg import NavSatFix
from drone.msg import GasSensorData
from drone.msg import DroneComm
from drone.msg import GridData
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

import FakeDroneSensor

#TODO convert all pos to np array form

'''[drone]----------------------------------------------------------------------
  Drone class, has an ID, name, sensors
  Holds representation of drone state, and has utilities to calculate desired
  reference states which are sent to px4/mavros.
----------------------------------------------------------------------------'''
class drone(threading.Thread):
  # Constructor. Requires start GPS location, exploration area size (in feet), grid resolution, and id
  def __init__(self, bounds, uav_id):
    super(drone, self).__init__()
    
    self.uav_id = uav_id
    self.moduleName = "drone_%d" % uav_id

    #threading variables
    self.daemon = True
    self.end_thread = False
    
    #TODO should rename this to air_qual_sensor
    #TODO should make case more consistent
    #sensors, publishers and subscribers
    #randomFlag: 0 = seeded random values, 1 = non-seeded random values
    #randomFlag = 0
    #self.gasSensor = FakeDroneSensor.FakeDroneSensor(bounds, resolutions, randomFlag, uav_id, 1, 3)
    
    #subscribe to topic where GPS values are published
    self.sub_gps = pos_gps_subscriber("uav%d/mavros/global_position/global" % uav_id)
    
    #subscribe to position reference from Michael's output
    self.sub_pos_ref = pos_ref_subscriber("uav%d/pos_ref" % uav_id)

    #publish position reference from Michael's output
    self.pub_pos_ref = pos_ref_publisher("uav%d/pos_ref" % uav_id)

    #publish velocity control reference
    self.pub_vel_ref = vel_ref_publisher("uav%d/vel_ref" % uav_id)

    #set explorable area bounds in terms of GPS locations
    self.bounds = bounds
    
    #init position using gps
    self.pos_gps = self.sub_gps.get_pos()
    self.x = self.pos_gps['longitude']
    self.y = self.pos_gps['latitude']
    self.z = self.pos_gps['altitude']

    #init pid controller
    self.pid = pid_controller()
    self.pid.pos = np.array([self.x, self.y, self.z])

    #init pois
    self.mission = np.zeros((1, 3))
    self.poi_index = 0
    self.mission_id = -1

  '''[callback]----------------------------------------------------------------
    Used to communicate with parent thread
  --------------------------------------------------------------------------'''
  def callback(self, msg):
    #instructs this thread to terminate loop
    if msg == "end":
      self.end_thread = True

  '''[update_state]------------------------------------------------------------
    Gets most recent readings from subscribed topics, update position in
    preparation for calculations on reference state.
  --------------------------------------------------------------------------'''
  def update_state(self):
    #use gps to determine current position
    self.pos_gps = self.sub_gps.get_pos()
    self.x = self.pos_gps['longitude']
    self.y = self.pos_gps['latitude']
    self.z = self.pos_gps['altitude']

    self.pid.pos = np.array([self.x, self.y, self.z])

    #check other subscribed topics to determine whether to update anything
    self.pos_ref = self.sub_pos_ref.get_pos()

  #TODO not sure how useful this is. leaving it in for now
  '''[in_bounds]---------------------------------------------------------------
    Checks whether drone is in the mission area or not.
    return - 1 if in mission area
             0 otherwise
  --------------------------------------------------------------------------'''
  def in_bounds(self):
    ''' Out of bounds check on exploration area
    if self.IsWithinSearchArea() == 0:
      if self.x < self.bounds['xMin'] or self.x > self.bounds['xMax']:
        print "[drone] [%d]: X out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['xMin'], self.bounds['xMax'], self.x)

      if self.y < self.bounds['yMin'] or self.y > self.bounds['yMax']:
        print "[drone] [%d]: Y out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['yMin'], self.bounds['yMax'], self.y)
      
      if self.z < self.bounds['zMin'] or self.z > self.bounds['zMax']:
        print "[drone] [%d]: Z out of exploration area [%.4f %.4f] %.5f" %(self.id, self.bounds['zMin'], self.bounds['zMax'], self.z)
    '''

    if ((self.x <= self.bounds['xMax'] + 0.5/111111.11) and (self.x >= self.bounds['xMin'] - 0.5/111111.11) and (self.y <= self.bounds['yMax'] + 0.5/111111.11) and (self.y >= self.bounds['yMin'] - 0.5/111111.11) and (self.z <= self.bounds['zMax'] + 0.5/111111.11) and (self.z >= self.bounds['zMin'] - 0.5/111111.11)):
      return 1
    else:
      return 0

  '''[run]---------------------------------------------------------------------
    Loop for drone thread
  --------------------------------------------------------------------------'''
  def run(self):

    #TODO might be worthwhile to listen to takeoff ROS topic to better handle
    #     takeoff and initialization

    #wait for GPS to initialize before using it to set objectives
    self.x = -1000
    
    print "[drone] [%d] Waiting for GPS to init" %(self.uav_id)
    
    while self.x < -180.0 or self.x > 180.0:
      self.update_state()
      time.sleep(1)

    print "[drone] [%d] Initialized GPS [%f %f %f]" % (self.uav_id, self.x, self.y, self.z)
    
    self.mission_id = -1
    while not self.end_thread:
      
      #TODO add mission-level state control for interactive inputs
      
      t_start = time.time()

      #update position based on GPS reading
      self.update_state()
      
      #publish a velocity vector so C++ module can interpolate and send to px4
      vel_ref = self.pid.vel_control_output(self.pos_ref)
      self.pub_vel_ref.publish(vel_ref)
    
      #print(self.uav_id, self.pos, pid_output, self.pid.kp * error, self.pid.ki * self.integral, self.pid.kd * derivative)
      
      #TODO publish an air quality sensor value
      #self.gasSensor.PublishGasSensorValue()

      t_end = time.time()
      
      #helps schedule threads more fairly if possible
      if t_end - t_start < 1.0:
        #print "[drone] [%d] Sleep: %f" %(self.id, t_end - t_start)
        time.sleep(1.0 - (t_end - t_start))

  

'''[pid_controller]------------------------------------------------------------
  PID controller for drone
----------------------------------------------------------------------------'''
class pid_controller:
  def __init__(self):
    self.pos = np.zeros((3))
    self.prev_error = np.zeros((3))
    self.integral = np.zeros((3))
    self.dt = 0.01
    self.kp = 0.8 * 2.0
    self.ki = 0
    self.kd = 0

  def vel_control_output(self, pos_ref):
    error = pos_ref - self.pos
    self.integral = self.integral * 0.99 + error * self.dt
    derivative = (error - self.prev_error) / self.dt

    self.prev_error = error
    pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    return pid_output

  def reset(self):
    self.prev_error = np.zeros((3))
    self.integral = np.zeros((3))



'''[pos_subscriber]------------------------------------------------------------
  Handles callbacks related to position subscriber and provides interface for
  accessing the associated position data.
----------------------------------------------------------------------------'''
class pos_gps_subscriber:
  def __init__(self, sub_name):
    self.sub_name = sub_name
    self.longitude = -1000
    self.latitude  = -1000
    self.altitude  = -1000
    self.sub = rospy.Subscriber(self.sub_name, NavSatFix, self.gps_sub_callback)

  #def init_sub(self):
  #  self.sub = rospy.Subscriber(self.sub_gps_name, NavSatFix, self.gps_sub_callback)

  def gps_sub_callback(self,msg):
    self.longitude = msg.longitude
    self.latitude  = msg.latitude
    self.altitude  = msg.altitude
    print "Received GPS location [%f %f %f]" %(msg.longitude, msg.latitude, msg.altitude)

  def get_pos(self):
    #print "Get location issued: location is now (%f|%f|%f)" %(self.longitude, self.latitude, self.altitude)
    return {
      'longitude':self.longitude,
      'latitude':self.latitude,
      'altitude':self.altitude,
      'x':self.longitude,
      'y':self.latitude,
      'z':self.altitude      
    }



#TODO For Michael's use
#the envisioned use case is to instantiate pos_ref_publisher("topicnamehere")
#then to publish, call set_pos with message contents, and/or pub_pos
#feel free to change long/lat/alt to x/y/z if it makes it easier to think about.
#I am using them for now since there is already a msg for GPS coords, NavSatFix
'''[pos_ref_publisher]---------------------------------------------------------
  Publishes a reference position for the drone PID controller to follow.
----------------------------------------------------------------------------'''
class pos_ref_publisher:
  def __init__(self, pub_name):
    self.pub_name = pub_name
    self.longitude = -1000
    self.latitude  = -1000
    self.altitude  = -1000
    self.pub = rospy.Publisher(self.pub_name, NavSatFix, queue_size=1)

  def set_pos(self, pos):
    self.longitude = pos['longitude']
    self.latitude = pos['latitude']
    self.altitude = pos['altitude']
    #print "Drone %d objective publisher: objective is now (%f|%f|%f)" %(self.id, self.longitude, self.latitude, self.altitude)

  def pub_pos(self):
    msg = NavSatFix()
    msg.longitude = self.longitude
    msg.latitude = self.latitude
    msg.altitude = self.altitude
    #print "Drone %d: publishing objective (%f|%f|%f) on %s" %(self.id, self.longitude, self.latitude, self.altitude, self.WaypointPublisherName)
    
    self.pub.publish(msg)



'''[pos_ref_subscriber]--------------------------------------------------------
  Subscribes to a reference position for the drone PID controller to follow.
----------------------------------------------------------------------------'''
class pos_ref_subscriber:
  def __init__(self, sub_name):
    self.sub_name = sub_name
    self.longitude = -1000
    self.latitude  = -1000
    self.altitude  = -1000
    self.mid = -1
    self.pub = rospy.Subscriber(self.sub_name, NavSatFix, self.pos_ref_callback)
    
  def pos_ref_callback(self, msg):
    self.longitude = msg.longitude
    self.latitude  = msg.latitude
    self.altitude  = msg.altitude
    self.mid = msg.mission_id

  def get_pos(self):
    return np.array([self.longitude, self.latitude, self.altitude])

'''[vel_ref_publisher]---------------------------------------------------------
  Publishes a reference velocity for C++ module to follow
----------------------------------------------------------------------------'''
class vel_ref_publisher:
  def __init__(self, pub_name):
    self.pub_name = pub_name
    self.linear = np.zeros((3))
    self.pub = rospy.Publisher(self.pub_name, Twist, queue_size=1)

  def set_pos(self, vel):
    self.linear[0] = vel[0]
    self.linear[1] = vel[1]
    self.linear[2] = vel[2]
    #print "Drone %d objective publisher: objective is now (%f|%f|%f)" %(self.id, self.longitude, self.latitude, self.altitude)

  def pub_pos(self):
    msg = Twist()
    msg.linear = self.linear
    self.pub.publish(msg)



#TODO need to refactor for consistent naming or take out if unnecessary
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



'''[dist]----------------------------------------------------------------------
  Calculates distance between two points
----------------------------------------------------------------------------'''
def dist(p1, p2):
  assert len(p1) == len(p2)

  total = 0
  for i in range(len(p1)):
    total += (p2[i] - p1[i])**2

  return total**(1/2)



'''[Older Classes]----------------------------------------------------------'''

#DEPRECATED - need to update all topics to be able to use
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

#DEPRECATED
# should change isClose to a euclidean distance measurer
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
