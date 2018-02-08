import rospy
import tf
import time #testing
#import numpy as np
from math import radians, copysign, sqrt, pow, pi, cos
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
global vel
x = 0 #global cooridates
y = 0
goal = Point #goal as point
goal.x = 1.0
print goal.x
global yaw
def goal1(msg):
    #goal.x = msg.x = 1.0#goal calback
    #goal.y = msg.y = 0.0#goal set when goal_sub is published

    print goal.x
    print goal.y
    return

def location_callback(msg):
	#conversion from quaterion to euler
	global q
    	q = ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    	euler = tf.transformations.euler_from_quaternion(q)
	yaw = euler[2]


def start():
    #node
    rospy.init_node('odometry_movement')

    #publishing
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
    vel = Twist()

    #subscribing
    sub = rospy.Subscriber('/odom', Odometry, location_callback)
    subgoal = rospy.Subscriber('/goal_sub', Point, goal1)
    rate = rospy.Rate(10)
    vel.linear.x = 0.3
    #calculation Block
    while q is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    #Block until shutdown
        x = cos(yaw)
        if((x == goal.x) & (y == goal.y)):
            vel.linear.x = 0
            print "yes!"
        else:
            #print x
            vel.linear.x = 0.6
        pub.publish(vel)
        rate.sleep() #sync with 10Hz rate
if __name__ == '__main__':  #begin with start() method
    start()
