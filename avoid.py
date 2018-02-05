#! /usr/bin/env python

import rospy
#import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def steering(msg):
    
    #print len(msg.ranges)
    #np.isnan(msg.ranges[719])
    #print msg.ranges[719]
    
    left = msg.ranges[:180]	
    #print 'left' + left
    front = msg.ranges[181:440]
    #print 'front' + front
    right = msg.ranges[441:]
    #print 'right' + right
    

    #
    if min(left) < 0.9 and min(front) >= 1.0 :
        # If it's close to the wall, go forward
        # and keep closing to the wall
        message = Twist(
            Vector3(0.1, 0, 0),
            Vector3(0, 0, 0.45)
        )
        
    if min(right) < 0.9 and min(front) >= 1.0 :
        # If it's close to the wall, go forward
        # and keep closing to the wall
        message = Twist(
            Vector3(0.1, 0, 0),
            Vector3(0, 0, -0.45)
        )
        
    elif min(left) < 0.5 :
        # If the robot is very close to the wall,
        # only rotates to the other side
        message = Twist(
            Vector3(0, 0, 0),
            Vector3(0, 0, -0.25)
        )
        
    elif min(right) < 0.5 :
        # If the robot is very close to the wall,
        # only rotates to the other side
        message = Twist(
            Vector3(0, 0, 0),
            Vector3(0, 0, 0.25)
        )
        
    elif min(left) < 2.0 and min(front) < 2.0 :
        # If it's closing to the wall,
        # slows the velocity and rotate agressively
        message = Twist(
            Vector3(0.1, 0, 0),
            Vector3(0, 0, 0.65)
        )
        
    elif min(right) < 2.0 and min(front) < 2.0 :
        message = Twist(
            Vector3(0.1, 0, 0),
            Vector3(0, 0, -0.65)
        )
        
    else :
        # Move forward
        message = Twist(
            Vector3(0.2,0,0),
            Vector3(0,0,0)
        )
    
    pub.publish(message)

#node
rospy.init_node('avoid_stuff')

#publishing 
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)

#subscribing
sub = rospy.Subscriber('/scan', LaserScan, steering)

#Block until shutdown
r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()
