#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan
#initialize values
x = 0.0
y = 0.0
th = 0.0
dist = 0.0
turn = 0.0
vel = 0.0

#Set the goal coordinates
goal = Point()
goal.x = 6
goal.y = 0

#set the vector increments for a 0.5m radius
def turn_options(sub_x, sub_y, index):
  turn = Point()
   if index == 1:
        sub_x = x + 0.13
        sub_y = y + 0.48
   if index == 2:
        sub_x = x + 0.32
        sub_y = y + 0.37
   if index == 3:
        sub_x = x + 0.45
        sub_y = y + 0.21
   if index == 4:
        sub_x = x + 0.5
        sub_y = y + 0
   if index == 5:
        sub_x = x + 0.45
        sub_y = y - 0.21
   if index == 6:
        sub_x = x + 0.32
        sub_y = y - 0.37
   if index == 7:
        sub_x = x + 0.13
        sub_y = y - 0.48

#BEGIN DWA(robotPose,robotGoal,robotModel)
#   desiredV = calculateV(robotPose,robotGoal)
#   laserscan = readScanner()
#   allowable_v = generateWindow(robotV, robotModel)
#   allowable_w  = generateWindow(robotW, robotModel)
#   for each v in allowable_v
#      for each w in allowable_w
#      dist = find_dist(v,w,laserscan,robotModel)
#      breakDist = calculateBreakingDistance(v)
#      if (dist > breakDist)  //can stop in time
#         heading = hDiff(robotPose,goalPose, v,w)
#         clearance = (dist-breakDist)/(dmax - breakDist)
#         cost = costFunction(heading,clearance, abs(desired_v - v))
#         if (cost > optimal)
#            best_v = v
#            best_w = w
#            optimal = cost
#    set robot trajectory to best_v, best_w
#END
