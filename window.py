#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi
from std_msgs.msg import Empty
from time import time

x = 0.0
y = 0.0
th = 0.0


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
