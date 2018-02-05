#! /usr/bin/env python

import rospy
#import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
def steering(msg):
    global vel
    #print len(msg.ranges)
    #np.isnan(msg.ranges[719])
    #print msg.ranges[719]

    left = msg.ranges[0:127]
    #print 'left' + left
    front = msg.ranges[128: 392]
    #print 'front' + front
    right = msg.ranges[393:511]
    #print 'right' + right

    #
    if min(left) < 0.3 and min(front) >= 0.5 :
        # If it's close to the wall, go forward
        # and keep closing to the wall
        vel.linear.x = 0.1
        vel.angular.z = 1
        print "left"
    if min(right) < 0.3 and min(front) >= 0.5 :
        # If it's close to the wall, go forward
        # and keep closing to the wall
        vel.linear.x = 0.1
        vel.angular.z = -1
        print "right"
    elif min(left) < 0.3 :
        # If the robot is very close to the wall,
        # only rotates to the other side
        vel.linear.x = 0
        vel.angular.z = 1
        print "very left"
    elif min(right) < 0.3 :
        # If the robot is very close to the wall,
        # only rotates to the other side
        vel.linear.x = 0
        vel.angular.z = -1
        print "very right"
    elif min(left) < 0.5 and min(front) < 0.5 :
        # If it's closing to the wall,
        # slows the velocity and rotate agressively
        vel.linear.x = 0.1
        vel.angular.z = 0.65
        print "almost"
    elif min(right) < 0.5 and min(front) < 0.5 :
        vel.linear.x = 0.1
        vel.angular.z = -1
        print "almost"
    elif min(front) < 0.5 :
        vel.linear.x = 0
        vel.angular.z = 1
        print "turn round"
    else :
        # Move forward
        vel.linear.x = 0.3
        vel.angular.z = 0
        print "go forth"
#node
rospy.init_node('avoid_stuff')

#publishing
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
vel = Twist()
#subscribing
sub = rospy.Subscriber('/scan', LaserScan, steering)

#Block until shutdown
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(vel)
    r.sleep()
