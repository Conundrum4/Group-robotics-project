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

def newOdon(msg):
    global x
    global y
    global th

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, th) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #th = atan2(y,x)
    th = th*(180/pi)
    #print th
rospy.init_node("speed_controller", anonymous = True)
sub = rospy.Subscriber("odom", Odometry, newOdon)
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)
speed = Twist()
#opub = rospy.Publisher("odom", Odometry, queue_size =1)
# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

# reset odometry (these messages take a few iterations to get through)
timer = time()
while time() - timer < 0.5:
    reset_odom.publish(Empty())
r = rospy.Rate(5)
turn = Odometry()
goal = Point()
goal.x = 5
goal.y = 6

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A

    #angle_to_goal = angle_to_goal*(180/pi)
    #angle = angle_to_goal - th
    #print angle
    if th < 90:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    if th >= 90:
        speed.linear.x = 0.5
        speed.angular.z = 0.0
    pub.publish(speed)
    r.sleep()
rospy.spin()
