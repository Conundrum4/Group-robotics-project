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
goal.x = 5
goal.y = 0
def find_angle(goal_x,goal_y, x, y):
    #obtain the x,y vector to goal
    inc_x = goal.x - x
    inc_y = goal.y - y
    #use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A
    #convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)
    return angle_to_goal

def find_goal(goalx,goaly,x,y):
    #obtain the vector distance to goal
    vect = sqrt(pow((goalx -x),2) + pow((goaly-y),2))
    print vect
    return ("vect: %s" %(vect))

def check_ranges(distance):
    if distance < 2:
        return None
    return True
def steering(data):
    global goal
    global dist
    front = data.ranges[128:392]
    dist = min(front)
    #one = left eight = right
    seven = data.ranges[100:145]
    six = data.ranges[146:207]
    five = data.ranges[208:240]
    four = data.ranges[241:271]
    three = data.ranges[272:304]
    two = data.ranges[305:366]
    one = data.ranges[367:412]
    arr = [one, two, three, four, five, six, seven]
    i = 0
    predict = Point()
    predict.x = predict.y = 0
    best_path = Point()
    best_path.x = best_path.y = 0
    best_count = 1
    for x in range(7):
        if check_ranges(min(arr[i]))==True:
            print i
            predict = predict_path(x, y, i)
            if best_path.x == best_path.y == 0:
                best_path.x = predict.x
                best_path.y = predict.y
            elif find_goal(goal.x, goal.y, predict.x, predict.y) < find_goal(goal.x,goal.y,best_path.x,best_path.y):
                best_path = predict
                best_count = i+1
        i = i + 1
    set_arc(best_count)
    print best_path
def set_arc(index):
    global vel
    global turn
    if index == 1:
        vel = 0.2
        turn = -0.15
    if index == 2:
        vel = 0.2
        turn = -0.1
    if index == 3:
        vel = 0.3
        turn = -0.05
    if index == 4:
        vel = 0.3
        turn = 0
    if index == 5:
        vel = 0.3
        turn = 0.05
    if index == 6:
        vel = 0.2
        turn = 0.1
    if index == 7:
        vel = 0.2
        turn = 0.15
def predict_path(x, y, index):
    pred = Point()
    if index == 1:
        y = y + 1.503
        x = x + 1.319
    if index == 2:
        y = y + 1
        x = x + 1.732
    if index == 3:
        y = y + 0.333
        x = x + 1.972
    if index == 4:
        y = y + 0
        x = x + 2
    if index == 5:
        y = y - 0.333
        x = x + 1.972
    if index == 6:
        y = y - 1
        x = x + 1.732
    if index == 7:
        y = y - 1.503
        x = x + 1.319
    pred.x = x
    pred.y = y
    return pred

def find_goal(goalx,goaly,x,y):
    #obtain the vector distance to goal
    return sqrt(pow((goalx -x),2) + pow((goaly-y),2))
#Odometry callback
def newOdon(msg):
#use global variables
    global x
    global y
    global th
#obtain the xy coordinates of the robot from odometry
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
#obtain the angle 'yaw' using a quaternion to euler converter
    (roll, pitch, th) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
#convert the angle to degrees
    #th = atan2(y,x)
    th = th*(180/pi)
    #print th
#set up nodes
rospy.init_node("speed_controller", anonymous = True)
sub = rospy.Subscriber("/odom", Odometry, newOdon)
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
r = rospy.Rate(10)
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
