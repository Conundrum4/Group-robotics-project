#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi
from std_msgs.msg import Empty
from time import time

#initialize values
x = 0.0
y = 0.0
th = 0.0

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
sub = rospy.Subscriber("odom", Odometry, newOdon)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)
speed = Twist()
#opub = rospy.Publisher("odom", Odometry, queue_size =1)
# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

# reset odometry values (these messages take a few iterations to get through)
timer = time()
while time() - timer < 0.5:
    reset_odom.publish(Empty())
r = rospy.Rate(5)
#Set the goal coordinates
goal = Point()
goal.x = 1.5
goal.y = 1.5

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
    if angle > 1:
        speed.linear.x = 0.0
        speed.angular.z = 0.15
    if angle <= 1:
        speed.linear.x = 0.25
        speed.angular.z = 0.0
#check if the bot has reached the goal
    if inc_x < 0.5 and inc_y < 0.5:
	speed.linear.x = 0
	speed.angular.z = 0
    pub.publish(speed)
    r.sleep()
rospy.spin()
