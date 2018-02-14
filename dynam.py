#! /usr/bin/env python
#This version will find the target eventually. it is unreliable for demonstrations
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt, cos, sin
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
goal.x = 3
goal.y = 0.0
sub_goal = Point()
sub_goal.x = 0.0
sub_goal.y = 0.0
best_path = Point()
def find_goal(goalx,goaly,xp,yp):
    #obtain the vector distance to goal
    vect = sqrt(pow((goalx -xp),2) + pow((goaly-yp),2))
    print ("vect: %s" %(vect))
    return vect

def check_ranges(distance):
    if distance < 1.5:
        return False
    return True
def a_to_d(sensordata): #convert the 0:511 range to degrees
    return ((sensordata*180)/511)-90

def d_to_a(degrees): #convert an angle to the 0:511 range
    return 511*(degrees+90)/180
def steering(data):
    global best_path
    global goal
    global dist
    front = data.ranges[128:392]
    dist = 0
    k = -90 #start at left
    start = d_to_a(k) #convert to sensor range units
    end = d_to_a(k+45) #45 degree range makes half a metre range
    predict = Point()
    predict.x = predict.y = 0
    best_path = sub_goal
    for x in range(135):
        temp = (min(data.ranges[start:end]))
	dist = temp
        if temp >= 0.5 and temp >= dist:
            #check if the range has more space
            angle = k
            dist = temp
            predict = predict_point((k+45)/2) #half way between k and k+45
            if(best_path.x == best_path.y == 0):
                best_path = predict
            if(find_goal(goal.x,goal.y,predict.x,predict.y)<= find_goal(goal.x,goal.y,best_path.x,best_path.y)):
                best_path = predict
        k = k+1
        start = d_to_a(k)
        end = d_to_a(k+45)

def predict_point(angle):
    pred = Point()
    pred.x = x + 0.5*sin(angle)
    pred.y = y + 0.5*cos(angle)
    return pred
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
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
scan_sub = rospy.Subscriber('/scan', LaserScan, steering)
# reset odometry values (these messages take a few iterations to get through)
timer = time()
while time() - timer < 0.5:
    reset_odom.publish(Empty())
    print "reset"
r = rospy.Rate(15)
#Main method
while not rospy.is_shutdown():
#obtain the x,y vector to goal
    print ("goal x: %s goal y: %s" %(goal.x-x, goal.y-y))
    inc_x = sub_goal.x - x
    inc_y = sub_goal.y - y
    print ("incx = %s - %s incy = %s - %s" %(sub_goal.x, x, sub_goal.y, y))
#use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A
#convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)
#find the difference between the angle of the bot and angle needed to turn
    angle = angle_to_goal - th
    print ("x: %s y: %s th: %s angletg:%s angle: %s" % (inc_x, inc_y,th,angle_to_goal, angle))
#check if the bot is within a suitable angle to the goal
    if angle > 2 or angle < -2:
        speed.linear.x = 0.0
        if(angle < -2):
            speed.angular.z = -0.5
        if angle > 2:
            speed.angular.z = 0.5
    if -2 <= angle <= 2:
        speed.linear.x = 0.1
        speed.angular.z = 0
#check if the bot has reached the goal
    if -0.1 < inc_x < 0.1 and -0.1 < inc_y < 0.1:
        if abs(sub_goal.x - goal.x)<= 0.1 and abs(sub_goal.y - goal.y)<=0.1:
            print  "met"
            r.sleep()
        else:
            sub_goal = best_path
    if -0.1 < inc_x < 0.1 and -0.1 < inc_y < 0.1:
	    speed.linear.x = 0
	    speed.angular.z = 0
    if dist < 0.2:
        speed.linear.x = 0
        print "nuh uh"
    pub.publish(speed)
    r.sleep()
rospy.spin()
