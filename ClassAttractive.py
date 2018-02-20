import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt, sin, cos
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan
import numpy as np

class Attractive:
        alpha = 0.1
        dg = 0
        thg = 0
	def __init__(self,Xg,Yg,Xc,Yc,Rg,Sg):
		self.Xg = Xg
		self.Yg = Yg
		self.Xc = Xc
		self.Yc = Yc
		self.Rg = Rg
		self.Sg = Sg

	def distance_to_goal(self):
		dg = (sqrt(pow(Xg-Xc),2)+(pow(Yg-Yc),2))
		return

	def angle_to_goal(self):
		thg =  atan2(Yg-Yc,Xg-Xc)
                return

	def update_coords(Xc,Yc):
		self.Xc = Xc
		self.Yc = Yc
		return

        def check_dist_goal():
            delta = Point()
        #goal statement
            if dg < Rg:
                delta.x = delta.y = 0
        #if within search field
            if Rg <= dg <= Sg + Rg:
                delta.x = alpha*(dg-Rg)*cos(thg)
                delta.y = alpha*(dg-Rg)*sin(thg)
        #if outside search field
            else:
                delta.x = alpha*Sg*cos(thg)
                delta.y = alpha*Sg*sin(thg)
            return delta

class Repulsive:
        beta = 0.1
        do = 0
        tho = 0

	def __init__(self,Xo,Yo,Xc,Yc,Ro,So):
		self.Xo = Xo
		self.Yo = Yo
		self.Xc = Xc
		self.Yc = Yc
		self.Ro = Ro
		self.So = So

	def distance_to_goal(self):
		do = (sqrt(pow(Xo-Xc),2)+(pow(Yo-Yc),2))
                return

	def angle_to_goal(self):
		tho =  atan2(Yo-Yc,Xo-Xc)
                return

	def update_coords(Xc,Yc):
		self.Xc = Xc
		self.Yc = Yc
		return

        def check_dist_goal():
            delta = Point()
            #goal statement
            if do < Ro:
                delta.x = delta.y = 0
            #if within search field
            if Ro <= do <= So + Ro:
                delta.x = beta*(do-Ro)*cos(thg)
                delta.y = beta*(do-Ro)*sin(thg)
            #if outside search field
            else:
                delta.x = beta*So*cos(thg)
                delta.y = beta*So*sin(thg)
            return delta
#Implementation

current_x = 0.0
current_y = 0.0
current_th = 0.0
goal = Point()
goal.x = -2
goal.y = 2
delta = Point()

def a_to_d(a):
    return (a/720)*180 -90

def steering(data):
    Fa = Attractive(goal.x,goal.y,current_x,current_y,0.3,0.2) #Attractive force
	#PUT DATA HERE
    Fr = [None]*10
    temp = Point()
    total_ranges = 720/10
    start = 0
    end = 0
    i = 0
    j = round(total_ranges,0)
    
    arr = 0
    index = []*10
    for i in range (11):
    
        start = i*j
        end = ((i+1)*j)-1
        arr = (min(data.ranges[int(start):int(end)]))
        index = (data.ranges[int(start):int(end)]).index(min(data.ranges[int(start):int(end)]))
        temp.x = current_x + arr*cos(a_to_d((start+end)/2))
        temp.y = current_y + arr*sin(a_to_d((start+end)/2))
        Fr[i] = Repulsive(temp.x,temp.y,current_x,current_y,0.2,0.3)
        print Fr[i].Xo

    for i in range(10):
        delta += Fr[i].check_dist_goal()
    delta += Fa.check_dist_goal()
    print A.Rg
def Odom(msg):
	global current_x
	global current_y
	global current_th
	current_x = msg.pose.pose.position.x                                        # Set global variable for x co-ord
	current_y = msg.pose.pose.position.y                                        # Set global variable for y co-ord
	roll = pitch = current_th = 0.0

	rot_q = msg.pose.pose.orientation
	#obtain the angle 'yaw' using a quaternion to euler converter
	(roll, pitch, current_th) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	#convert the angle to degrees

	current_th = current_th*(180/pi)                                            # Set global variable for angle (in degrees)

#set up nodes
rospy.init_node("speed_controller", anonymous = True)                           # Node
sub = rospy.Subscriber("/odom", Odometry, Odom)                              # Odometry subscriber
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)        # Publisher to move robot
speed = Twist()

# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)  # Publisher to reset the Odometry
scan_sub = rospy.Subscriber('/scan', LaserScan, steering)                       # Subscriber to get info from the laser

# reset odometry values (these messages take a few iterations to get through)
timer = time()
#the longer the timer set the more accurate the odometry initializes
while time() - timer < 1.5:                                                     # 1.5 second delay.  This seems to improve odometry accuracy on reset
	reset_odom.publish(Empty())
	resetted = True
	r = rospy.Rate(10)

#Main method
while not rospy.is_shutdown():
#obtain the x,y vector to goal
        inc_x = delta.x - current_x
    	inc_y = delta.y - current_y
#use tan to find the angle needed to turn towards the goal
    	angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A

#convert angle to degrees
    	angle_to_goal = angle_to_goal*(180/pi)

#find the difference between the angle of the bot and angle needed to turn
    	angle = angle_to_goal - current_th
    	print ("x: %s y: %s th: %s" % (current_x, current_y, current_th))

# 4.5 degree error is a comprimise between speed and accuracy
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

# check if the bot is within a suitable angle to the goal
    	if -0.1 <= inc_x <= 0.1 and -0.1 <= inc_y <= 0.1:
	    speed.linear.x = 0
	    speed.angular.z = 0
            print 'I am here!'
            achieved = True                                                     # Updates the achieved boolean

        pub.publish(speed)
    	r.sleep()
rospy.spin()

    #GAZEBO 720 left to right
#nine = data.ranges[648:719]
#eight = data.ranges[576:647]
#seven = data.ranges[504:575]
#six = data.ranges[432:503]
#five = data.ranges[360:431]
#four = data.ranges[288:359]
#three = data.ranges[216:287]
#two = data.ranges[144:215]
#one = data.ranges[72:143]
#zero = data.ranges[0:71]

    #ROBOT 512 right to left
#zero = data.ranges[460:511]
#one = data.ranges[407:459]
#two = data.ranges[358:408]
#three = data.ranges[307:357]
#four = data.ranges[256:306]
#five = data.ranges[205:255]
#six = data.ranges[154:204]
#seven = data.ranges[103:153]
#eight = data.ranges[52:102]
#nine = data.ranges[0:51]

