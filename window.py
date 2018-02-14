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
global current_x = 0.0 #current x co-ord of robot
global current_y = 0.0 #current y co-ord of robot
global current_th = 0.0 #current orientation of the robot
dist = 0.0
turn = 0.0
vel = 0.0

#Set the goal coordinates
global goal = Point()
goal.x = 6
goal.y = 0

global sub_goal = point()
sub_goal.x = 0
sub_goal.y = 0

#set the vector increments for a 0.5m radius
def turn_options(index):
  turn = Point()
  x = 0
  y = 0
   if index == 1:
        x = current_x + 0.13
        y = current_y + 0.48
   if index == 2:
        x = current_x + 0.32
        y = current_y + 0.37
   if index == 3:
        x = current_x + 0.45
        y = current_y + 0.21
   if index == 4:
        x = current_x + 0.5
        y = current_y + 0
   if index == 5:
        x = current_x + 0.45
        y = current_y - 0.21
   if index == 6:
        x = current_x + 0.32
        y = current_y - 0.37
   if index == 7:
        x = current_x + 0.13
        y = current_y - 0.48
   
   turn.x = x
   turn.y = y
   return turn
        
#Obtain the shortest distance to the goal for a paticular set of co-ords
def find_goal(x,y):
    vect = sqrt(pow((goal.x -x),2) + pow((goal.y-y),2))
    #print ("Distance to goal: %s" %(vect))
    return vect
        
#Is there an obstacle within 0.7m of the robot on a chosen path?  If yes, eliminate that path as an option
def check_ranges(distance):
    if distance < 0.7:
        return False
    return True
  
#set the ranges attributed to each option
def steering(data):
    
    #In gazebo left to right, on robot right to left
    six = data.ranges[608:719]
    five = data.ranges[509:608]
    four = data.ranges[409:508]
    three = data.ranges[313:408]
    two = data.ranges[213:312]
    one = data.ranges[113:212]
    zero = data.ranges[0:112]

    #an array of the ranges
    arr = [one, two, three, four, five, six, seven]
    i = 0
    
    closest = [0,0,0,0,0,0,0]
    
    for i in range(7)
      if check_ranges(min(arr[i]))==True:
        closest[i] = find_goal
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
