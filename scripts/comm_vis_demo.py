#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import numpy as np

import random
import sys
import time

import rospy
from drone.msg import DroneComm

util_bitrates = []
avail_bitrates = []
times = []

def animate(i):
  #generate random placeholder avail/utilized bitrates
  avail_bitrate_sample = random.randint(0, 5000)
  util_bitrate_sample = random.randint(0, avail_bitrate_sample)
  
  #plot these over time
  time_sample = time.time() - start

  times.append(time_sample)
  avail_bitrates.append(avail_bitrate_sample)
  util_bitrates.append(util_bitrate_sample)

  ax1.clear()
  ax1.plot(times, avail_bitrates, label="Available byterate")
  ax1.plot(times, util_bitrates, label="Utilized byterate")
  ax1.set_xlabel("Time (s)")
  ax1.set_ylabel("Byterate (b/s)")
  ax1.legend()

if __name__ == "__main__":
  fig = plt.figure()
  ax1 = fig.add_subplot(1,1,1)

  start = time.time()
  ani = animation.FuncAnimation(fig, animate, interval=100)
  plt.show()
