#! /usr/bin/env python

import rospy
#import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def steering(msg):
    
    #print len(msg.ranges)
    #np.isnan(msg.ranges[719])
    #print msg.ranges[719]
    
    left = msg.ranges[180:]	
    #print 'left' + left
    front = msg.ranges[351:370]
    #print 'front' + front
    right = msg.ranges[:441]
    #print 'right' + right
    


    #If left is closer than right, turn right
    if min(left) < min(right) and min(front) >= 2.0:
        
        message = Twist(Vector3(0.1,0,0), Vector3(0,0,-1.0))
    
    #If right is closer than left, turn left
    if min(right) < min(left) and min(front) >= 3.0:
        
        message = Twist(Vector3(0.1,0,0), Vector3(0,0,1.0))
    
    #If left is less than 1.5m from the wall
    elif min(left) < 0.5 and min(front) < 0.5:
        
        message = Twist(Vector3(0,0,0), Vector3(0,0,-1.0))
    
    #If right is less than 1.5m from the wall
    elif min(right) < 0.5 and min(front) < 0.5:
        
        message = Twist(Vector3(0,0,0), Vector3(0,0,-1.0))
        
    else:
        
        message = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
    
    
    pub.publish(message)

#node
rospy.init_node('avoid_stuff')

#publishing 
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)

#subscribing
sub = rospy.Subscriber('/scan', LaserScan, steering)

#Block until shutdown
rospy.spin()
