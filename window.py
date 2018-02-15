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
   if index == 0:
        x = current_x + 0.13
        y = current_y + 0.48
   if index == 1:
        x = current_x + 0.32
        y = current_y + 0.37
   if index == 2:
        x = current_x + 0.45
        y = current_y + 0.21
   if index == 3:
        x = current_x + 0.5
        y = current_y + 0
   if index == 4:
        x = current_x + 0.45
        y = current_y - 0.21
   if index == 5:
        x = current_x + 0.32
        y = current_y - 0.37
   if index == 6:
        x = current_x + 0.13
        y = current_y - 0.48
   
   turn.x = x
   turn.y = y
   return turn
        
#Obtain the shortest distance to the goal for a paticular set of co-ords
def dist_to_goal(x,y):
    vect = sqrt(pow((goal.x -x),2) + pow((goal.y-y),2))
    #print ("Distance to goal: %s" %(vect))
    return vect
        
#Is there an obstacle within 0.7m of the robot on a chosen path?  If yes, eliminate that path as an option
def check_ranges(distance):
    if distance < 0.7:
        return False
    return True

#Find the x and y co-ords of the goal
def find_angle(goal_x,goal_y, x, y):
    inc_x = goal.x - x
    inc_y = goal.y - y
    #use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A
    #convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)
    return angle_to_goal
   
  
#set the ranges attributed to each option
def steering(data):
    
    laser_ranges = [] #an array to store the range values
    new_coords = [] #an array to store potential new co-ords to move to
    no_obstruction = [] #an array to store viable new co-ords (no obstruction present)
    closest = [] #an array to store the distances of the new co-ords from the goal
    
    #In gazebo left to right, on robot right to left
    six = data.ranges[608:719]
    five = data.ranges[509:608]
    four = data.ranges[409:508]
    three = data.ranges[313:408]
    two = data.ranges[213:312]
    one = data.ranges[113:212]
    zero = data.ranges[0:112]

    #an array of the ranges
    laser_ranges = [one, two, three, four, five, six, seven]
    i = 0
    j = 0
    k = 0
    
    
    for i in range(7)
      new_coords.append(turn_options[i])                                              #adds the new co-ords to the array
      closest.append(dist_to_goal(turn_options[i].x, dist_to_goal(turn_options[i].y)) #adds distance to goal
        
          if min(laser_ranges[i]) < 0.7                                               #checks if there is an obstruction
              no_obstruction.append(1)                                                #This is a viable option
          else
               no_obstruction.append(0)                                               #There is an obstruction present
    
     for j in range(7)
       if laser_ranges[i] == 1 and closest[i] == min(closest)                         #checks laser ranges and dist to goal
           sub_goal = new_coords[i]                                                   #sets subgoal to co-ords with no obstructions...
       else                                                                           #...is closest to the goal
           print 'nowhere to go!'
         
                     
                     
#Odometry callback
def newOdom(msg):
    
    current_x = msg.pose.pose.position.x     #set global variable
    current_y = msg.pose.pose.position.y     #set global variable
                     
    orient = msg.pose.pose.orientation       #local variable
    th = 0.0
    (roll, pitch, th) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    
    current_th = th *(180/pi)                 #set global variable and convert from radians to degrees
     
#set up nodes
rospy.init_node("speed_controller", anonymous = True)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)
speed = Twist()

#opub = rospy.Publisher("odom", Odometry, queue_size =1)
# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
scan_sub = rospy.Subscriber('/scan', LaserScan, steering) 
                     
# reset odometry values (these messages take a few iterations to get through)
timer = time()
while time() - timer < 0.25:
    reset_odom.publish(Empty())
r = rospy.Rate(0.5)

                     
                     
                     
#Main method
while not rospy.is_shutdown():

#obtain the x,y vector to goal
    inc_x = goal.x - x
    inc_y = goal.y - y

#use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A

#convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)

#find the difference between the angle of the bot and angle needed to turn
    angle = angle_to_goal - th
    print ("x: %s y: %s" % (inc_x, inc_y))

#check if the bot is within a suitable angle to the goal
    if angle > 1 or angle < -1:
        speed.linear.x = 0.0
        if(angle < -1):
            speed.angular.z = -0.5
        if angle > 1:
            speed.angular.z = 0.5
    if -1 <= angle <= 1:
        print ("vel: %s turn: %s" %(vel,turn))
        speed.linear.x = 0.3
        speed.angular.z = 3*turn
#check if the bot has reached the goal
    if -0.1 < inc_x < 0.1 and -0.1 < inc_y < 0.1:
	    speed.linear.x = 0
	    speed.angular.z = 0
    if dist < 0.2:
        speed.linear.x = 0
        print "nuh uh"
    pub.publish(speed)
    r.sleep()
rospy.spin()                     
                     
                     
                     
                     
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
