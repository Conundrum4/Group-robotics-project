import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt, sin, cos
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan
import numpy as np

# A class to calculate the attractive force towards the goal
class Attractive:
	def __init__(self,Xg,Yg,Xc,Yc,Rg,Sg):
		self.Xg = Xg                 # x co-ord of goal
		self.Yg = Yg                 # y co-ord of goal
		self.Xc = Xc                 # x co-ord of robot
		self.Yc = Yc                 # y co-ord of robot
		self.Rg = Rg                 # radius of the goal
		self.Sg = Sg                 # radius of the attractive field around the goal

        # calculates the angle to the goal, returns thg
	def angle_to_goal(self):
		thg =  atan2(self.Yg-self.Yc,self.Xg-self.Xc)
                return thg
        
	# updates the robots current co-ordinates
	def update_coords(self,Xc,Yc):
		self.Xc = Xc
		self.Yc = Yc
		return
	
        # calculates the distance to the goal and returns the attractive force, delta
        def check_dist_goal(self):
            delta = Point()
            alpha = 0.4
            #goal statement
            dg = sqrt(pow((self.Xg-self.Xc),2)+pow((self.Yc-self.Yg),2))
            thg = atan2(self.Yg-self.Yc,self.Xg-self.Xc)
            if dg < self.Rg:
                delta.x = delta.y = 0
            #if within search field
            if self.Rg <= dg <= self.Sg + self.Rg:
                delta.x = alpha*(dg-self.Rg)*cos(thg)
                delta.y = alpha*(dg-self.Rg)*sin(thg)
            #if outside search field
            if dg > self.Sg + self.Rg:
                delta.x = alpha*self.Sg*cos(thg)
                delta.y = alpha*self.Sg*sin(thg)
            return delta

# A class to calculate the repulsive forces for each obstacle
class Repulsive:
	def __init__(self,Xo,Yo,Xc,Yc,Ro,So):
		self.Xo = Xo                 # x co-ord of the obstacle
		self.Yo = Yo                 # y co-ord of the obstacle
		self.Xc = Xc                 # x co-ord of the robot
		self.Yc = Yc                 # y co-ord of the robot
		self.Ro = Ro                 # radius of the obstacle (allows for a safety margin)
		self.So = So                 # radius of the repulsive field around the obstacle

	# calculates the angle to the obstacle and returns tho
	def angle_to_goal(self):
		tho =  atan2(self.Yo-self.Yc,self.Xo-self.Xc)
                return tho
   
        # updates the robots current co-ordinates
	def update_coords(self,Xc,Yc):
		self.Xc = Xc
		self.Yc = Yc
		return

	# calculates the distance to the obstacle and returns the repulsive force, delta 
        def check_dist_goal(self):
            delta = Point()
            beta = 0.3
            #goal statement
            do = sqrt(pow((self.Xo-self.Xc),2)+pow((self.Yc-self.Yo),2))
            tho = atan2(self.Yo-self.Yc,self.Xo-self.Xc)
            if do < self.Ro:
                delta.x = -sign(cos(tho))*5000
                delta.y = -sign(sin(tho))*5000
            #if within search field
            if self.Ro <= do <= self.So + self.Ro:
                delta.x = -beta*(self.So+self.Ro-do)*cos(tho)
                delta.y = -beta*(self.So+self.Ro-do)*sin(tho)
            #if outside search field
            else:
                delta.x = delta.y = 0
            return delta


#Implementation

current_x = 0.0          # x co-ord of the robot (global)
current_y = 0.0          # y co-ord of the robot (global)
current_th = 0.0         # orientation of the robot (global)
goal = Point()           # goal co-ordinates (global)
goal.x = 6               # x co-ord of the goal (global)
goal.y = 0               # y co-ord of the goal (global)
delta = Point()          # vector of the robot (global)
delta.x = delta.y = 0    # set it to zero 
resetted = False         # Has the odometry been reset? Boolean (global)
achieved = True          # Has the robot reached its goal? Boolean (global)

def a_to_d(sensordata): #convert the 0:511 range to degrees
    return ((sensordata*180)/511)-90

def d_to_a(degrees): #convert an angle to the 0:511 range
    return 511*(degrees+90)/180

def steering(data):
    global delta
    global goal
    global current_x
    global current_y
    global resetted

    if(achieved == False):
	return

    delta.x = delta.y = 0
    if(resetted == False):
        return

    Fa = Attractive(goal.x,goal.y,current_x,current_y,0.3,0.3) #Attractive force
    
    laser = np.asarray(data.ranges) # convert laser ranges to np array
    laser = np.nan_to_num(laser) # change all nan values to 0
    laser = np.where(laser == 0, data.range_max + 10, laser) # laser is temp array that converts nan values to maximum range
    laser = np.where(laser > 30, data.range_max + 10, laser) # nan is where the distance is outwith range
	
    Fr = [None]*10
    temp = Point()
    total_ranges = 720/10
    k = -90
    i = 0
    j = 72

    end = ((i+1)*j)-1
    nine = laser[648:719]
    eight = laser[576:647]
    seven = laser[504:575]
    six = laser[432:503]
    five = laser[360:431]
    four = laser[288:359]
    three = laser[216:287]
    two = laser[144:215]
    one = laser[72:143]
    zero = laser[0:71]
    senses = [zero,one,two,three,four,five,six,seven,eight,nine]
    index = 0
    for i in range(10):
        if not (senses[i].size):
            continue
        arr = (min(senses[i]))
        temp.x = current_x + arr*cos(i*18+9)
        temp.y = current_y + arr*sin(i*18+9)
        Fr[i] = Repulsive(temp.x,temp.y,current_x,current_y,0.4,0.25)
        print Fr[i].check_dist_goal()
    i = 0
    for i in range(10):
        if (Fr[i] == None):
            continue
        delta.x = delta.x + Fr[i].check_dist_goal().x
        delta.y = delta.y + Fr[i].check_dist_goal().y
    delta.x = delta.x + Fa.check_dist_goal().x
    delta.y = delta.y + Fa.check_dist_goal().y
    achieved = False
    print "DELTA: %s" %(delta)
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
while time() - timer <1.5:                                                     # 1.5 second delay.  This seems to improve odometry accuracy on reset
	reset_odom.publish(Empty())
	resetted = True
	r = rospy.Rate(10)

#Main method
while not rospy.is_shutdown():
#obtain the x,y vector to goal
        inc_x = delta.x - current_x
    	inc_y = delta.y - current_y
        vel = sqrt(pow(delta.x,2)+pow(delta.y,2))
#use tan to find the angle needed to turn towards the goal
    	angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A

#convert angle to degrees
    	angle_to_goal = angle_to_goal*(180/pi)
#find the difference between the angle of the bot and angle needed to turn
    	angle = angle_to_goal - current_th
        angle = (angle+180)%360 -180
    	print ("x: %s y: %s th: %s" % (current_x, current_y, current_th))
        turn = ((pi*angle)/180)
# 4.5 degree error is a comprimise between speed and accuracy
    	if angle > 2 or angle < -2:
      	    print angle
            speed.linear.x = min(vel,0)
            if(angle < -2):
          	  speed.angular.z = -0.3
            if angle >= 2:
          	  speed.angular.z = 0.3
    	elif -2 <= angle <= 2:
       	    speed.linear.x = min(vel,.4)
            speed.angular.z = 0.0

# check if the bot is within a suitable angle to the goal
	while abs(int_x)<=0.25 and abs(int_y)<=0.25:
	        speed.linear.x = 0
	        speed.angular.z = 0
		achieved == True
    	while abs(goal.x-current_x)<=0.25 and abs(goal.y-current_y)<=0.25:
	        speed.linear.x = 0
	        speed.angular.z = 0
            #print 'I am here!'
            #achieved = True                                                     # Updates the achieved boolean

        pub.publish(speed)
    	r.sleep()
rospy.spin()
