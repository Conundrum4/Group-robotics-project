#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt, cos, sin
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan
import numpy as np

#initialize values
current_x = 0.0                                                                 #current x co-ord of robot
current_y = 0.0                                                                 #current y co-ord of robot
current_th = 0.0                                                                #current orientation of the robot
dist = 0.0
turn = 0.0
vel = 0.0
achieved = True
resetted = False
#Set the goal coordinates
goal = Point()
goal.x = 6
goal.y = 2

sub_goal = Point()
sub_goal.x = 0
sub_goal.y = 0

#set the vector increments for a 0.5m radius
def turn_options(index):
  global current_x
  global current_y
  global turn 
  turn = Point()
  angle = 0
  x = 0
  y = 0
  if index == 0:
        angle = 90
  if index == 1:
        angle = 60
  if index == 2:
       angle = 30
  if index == 3:
        angle = 0
  if index == 4:
       angle = -30
  if index == 5:
       angle = -60
  if index == 6:
        angle = -90
  if index == 7:
        angle = -120
  if index == 8:
        angle = -150
  if index == 9:
        angle = 180
  if index == 10:
       angle = 150
  if index == 11:
	angle = 120
  if index == 12:
       angle = 90
  turn.x = x + 0.5*cos(angle+current_th)
  turn.y = y + 0.5*sin(angle+current_th)
  return turn

#Obtain the shortest distance to the goal for a paticular set of co-ords
def dist_to_goal(x,y):
    global goal
    vect = sqrt(pow((goal.x -x),2) + pow((goal.y-y),2))
    #print ("Distance to goal: %s" %(vect))
    return vect

#Is there an obstacle within 0.7m of the robot on a chosen path?  If yes, eliminate that path as an option
def check_ranges(distance):
    if distance < 1.2:
        return False
    return True

#Find the x and y co-ords of the goal
def find_angle(goal_x,goal_y, x, y):
    global goal
    inc_x = goal.x - x
    inc_y = goal.y - y
    #use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A
    #convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)
    return angle_to_goal


#set the ranges attributed to each option
def steering(data):
    global sub_goal
    global achieved
    #1000 is dummy value so that a value is not considered
    laser_ranges = [None]*7                                                           #an array to store the range values
    new_coords = [None]*7                                                             #an array to store potential new co-ords to move to
    no_obstruction = [0]*7                                                         #an array to store viable new co-ords (no obstruction present)
    closest = [50]*7                                                               #an array to store the distances of the new co-ords from the goal

    #In gazebo left to right(0 -> 6), on robot right to left (6 <- 0)
    laser = np.asarray(data.ranges)
    laser = np.nan_to_num(laser)
    laser = np.where(laser == 0, data.range_max + 10, laser) #laser is temp array that converts nan values to maximum range
    laser = np.where(laser > 30, data.range_max + 10, laser) #nan is where the distance is outwith range
    six = laser[608:719]
    five = laser[509:608]
    four = laser[409:508]
    three = laser[313:408]
    two = laser[213:312]
    one = laser[113:212]
    zero = laser[0:112]

    #an array of the ranges
    laser_ranges = [zero, one, two, three, four, five, six]
    i = 0
    j = 0

    if(achieved == False):
            #print 'I am not there yet!'
	    return

    if(resetted == False):
	    return
    for i in range(7):
	if(goal.x < 0):
		new_coords[i] = turn_options(i+6)
	else:
		new_coords[i] = turn_options(i)                                        #adds the new co-ords to the array
        closest[i] = dist_to_goal(turn_options(i).x, turn_options(i).y)        #adds distance to goal

        if min(laser_ranges[i]) > 1.2:                                          #checks if there is an obstruction
            no_obstruction[i] = 1                                            #This is a viable option
        else:
            no_obstruction[i] = 0
            closest[i] = 20*closest[i]
            print no_obstruction                                        #*20 to make sure that obstructed co-ords are not seen as closest
	    print closest
                                                                                #There is an obstruction present

    for j in range(7):
        if (no_obstruction[j] == 1) and (closest[j] == min(closest)):
            print j                                                             #checks laser ranges and dist to goal
            sub_goal = new_coords[j]
            print 'goals'
            print sub_goal.x
            print sub_goal.y
            achieved = False                                              #sets subgoal to co-ords with no obstructions...
        else:                                                                   #...is closest to the goal
            print 'nowhere to go!'


#Odometry callback
def newOdom(msg):
    global current_x
    global current_y
    global current_th
    current_x = msg.pose.pose.position.x     #set global variable
    current_y = msg.pose.pose.position.y     #set global variable
    roll = pitch = current_th = 0.0

    rot_q = msg.pose.pose.orientation
#obtain the angle 'yaw' using a quaternion to euler converter
    (roll, pitch, current_th) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
#convert the angle to degrees
    #th = atan2(y,x)
    current_th = current_th*(180/pi)
#set up nodes
rospy.init_node("speed_controller", anonymous = True)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)
speed = Twist()

# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
scan_sub = rospy.Subscriber('/scan', LaserScan, steering)

# reset odometry values (these messages take a few iterations to get through)
timer = time()
#the longer the timer set the more accurate the odometry initializes
while time() - timer < 1.5:
    reset_odom.publish(Empty())
resetted = True
r = rospy.Rate(10)

#Main method
while not rospy.is_shutdown():

#obtain the x,y vector to goal
    inc_x = sub_goal.x - current_x
    inc_y = sub_goal.y - current_y
#use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A

#convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)

#find the difference between the angle of the bot and angle needed to turn
    angle = (angle_to_goal-current_th)

    print ("x: %s y: %s th: %s" % (current_x, current_y, current_th))
    
#check if the bot is within a suitable angle to the goal
#4.5 degree error is a comprimise between speed and accuracy
    if angle > 4.5 or angle < -4.5:
        print angle
        speed.linear.x = 0.0
        if(angle < -4.5):
            speed.angular.z = -0.15
        if angle >= 4.5:
            speed.angular.z = 0.15
    elif -4.5 <= angle <= 4.5:
        speed.linear.x = 0.3
        speed.angular.z = 0.0
#check if the bot has reached the goal
    if -0.1 <= inc_x <= 0.1 and -0.1 <= inc_y <= 0.1:
	    speed.linear.x = 0
	    speed.angular.z = 0
            print 'I am here!'
            achieved = True
    
    pub.publish(speed)
    r.sleep()
rospy.spin()