import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt, sin, cos
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import *

# A class to calculate the attractive force toward the goal
class Attractive:
        def __init__(self,Xg,Yg,Xc,Yc,Rg,Sg):
            self.Xg = Xg                     # x co-ord of the goal
            self.Yg = Yg                     # y co-ord of the goal
            self.Xc = Xc                     # x co-ord of the robot
            self.Yc = Yc                     # y co-ord of the robot
            self.Rg = Rg                     # radius of the goal
            self.Sg = Sg                     # radius of the attractive field

        # calculate the distance to the goal, returns delta
        def check_dist_goal(self):
            delta = Point()
            alpha = 300                  # a constant for the attractive field

            #goal statement
            dg = sqrt(pow((self.Xg-self.Xc),2)+pow((self.Yg-self.Yc),2))
            thg = atan2(self.Yg-self.Yc,self.Xg-self.Xc)
            if dg < self.Rg:
                delta.x = delta.y = 0
            #if within search field
            if self.Rg <= dg <= self.Sg + self.Rg:
                delta.x = alpha*pow((dg-self.Rg),2)*cos(thg)
                delta.y = alpha*pow((dg-self.Rg),2)*sin(thg)
            #if outside search field
            if dg > self.Sg + self.Rg:
                delta.x = alpha*self.Sg*cos(thg)
                delta.y = alpha*self.Sg*sin(thg)
            return delta

# A class to calculate the repulsive force from obstacles
class Repulsive:
        def __init__(self,do,tho,Xc,Yc,Ro,So):
            self.do = do                   # distance to obstacle
            self.tho = tho                 # angle to obstacle
            self.Xc = Xc                   # x co-ord of the robot
            self.Yc = Yc                   # y co-ord of the robot
            self.Ro = Ro                   # radius of object (provides a safety margin)
            self.So = So                   # radius of the repulsive field

        # calculate the distance to an obstacle, returns delta
        def check_dist_goal(self):
            delta = Point()
            beta = 500                    # a constant for the repulsive field
            #goal statement
            if self.do < self.Ro:
                delta.x = -1*(cos(self.tho))*10000    # repulsion becomes very large if too close to object
                delta.y = -1*(sin(self.tho))*10000    # repulsion becomes very large if too close to object
            #if within search field
            if self.Ro <= self.do <= self.So + self.Ro:
                delta.x = -beta*pow((self.So+self.Ro-self.do),2)*cos(self.tho)
                delta.y = -beta*pow((self.So+self.Ro-self.do),2)*sin(self.tho)
            #if outside search field
            if self.do > self.So + self.Ro:
                delta.x = delta.y = 0
            return delta
#Implementation

current_x = 0.0                      # current x co-ord of the robot (global)
current_y = 0.0                      # current y co-ord of the robot (global)
current_th = 0.0                     # current orientation of the robot (global)
goal = Point()                       # goal co-ordinates (global)
goal.x = 5
goal.y = 5
delta = Point()                      # delta (global)
delta.x = delta.y = 0
resetted = True                     # Has the odometry been rest? Boolean (global)

# The steering function to move the robot
def steering(data):
    global delta
    global goal
    global current_x
    global current_y
    global resetted

    # Checks that the odometry hsas been reset
    if(resetted == False):
        return

    Fa = Attractive(goal.x,goal.y,current_x,current_y,0.5,20) # Attractive force
    laser = np.asarray(data.ranges)                           # Converts to a np array
    laser = np.nan_to_num(laser)                              # changes all nan values to 0
    laser = np.where(laser == 0, data.range_max + 10, laser)  # laser is temp array that converts 0 (nan) values to maximum range
    laser = np.where(laser > 30, data.range_max + 10, laser)  # nan is where the distance is outwith range

    # Assignments for the laser
    Fr = [0]*10
    temp = Point()
    i = 0
    senses = [[0]]*10

    #FOR GAZEBO
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

    #FOR ROBOT
    #zero = laser[460:511]
    #one = laser[407:459]
    #two = laser[358:408]
    #three = laser[307:357]
    #four = laser[256:306]
    #five = laser[205:255]
    #six = laser[154:204]
    #seven = laser[103:153]
    #eight = laser[52:102]
    #nine = laser[0:51]

    th = -0.5*pi                                                 # adds 90 degrees to th to make vectors curve around obstacle

    senses = [zero,one,two,three,four,five,six,seven,eight,nine] # an array of the laser assignments
    index = 0
    delta.x = delta.y = 0

    for i in range(10):
        arr = (min(senses[i]))                                   # the closest obstacle in this range
        th = (((i*18)+9)-90)*(pi/180)                            # the middle angle of the range

        temp.x = current_x + arr*cos(current_th+th)
        temp.y = current_y + arr*sin(current_th+th)
        tho = atan2(current_y-temp.y,current_x-temp.x)

        if(tho < 0):
            tho = tho + (0.3*pi) #putting the vector at a tangent to the obstacle
        else:
            tho = tho - (0.5*pi)
        Fr = Repulsive(arr, tho,current_x,current_y,0.5,1.2)
        delta.x = delta.x + Fr.check_dist_goal().x
        delta.y = delta.y + Fr.check_dist_goal().y
        print "arr: %s at %s" %(arr,tho)
        print "FRx: %s FRy: %s" %(Fr.check_dist_goal().x,Fr.check_dist_goal().y)

    delta.x = delta.x + Fa.check_dist_goal().x
    delta.y = delta.y + Fa.check_dist_goal().y
    print "DELTA: %s" %(delta)

# odometry function to retrieve position of robot
def Odom(msg):
    global current_x
    global current_y
    global current_th

    # Checks that the odometry has been reste
    if(resetted == False):
        return

    current_x = msg.pose.pose.position.x                                        # Set global variable for x co-ord
    current_y = msg.pose.pose.position.y                                        # Set global variable for y co-ord
    roll = pitch = current_th = 0.0

    rot_q = msg.pose.pose.orientation
    #obtain the angle 'yaw' using a quaternion to euler converter
    (roll, pitch, current_th) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #convert the angle to degrees



#set up nodes
rospy.init_node("speed_controller", anonymous = True)                           # Node
sub = rospy.Subscriber("/odom", Odometry, Odom)                                 # Odometry subscriber
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)        # Publisher to move robot
speed = Twist()

# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)  # Publisher to reset the Odometry
scan_sub = rospy.Subscriber('/scan', LaserScan, steering)                       # Subscriber to get info from the laser

# reset odometry values (these messages take a few iterations to get through)
timer = time()
#the longer the timer set the more accurate the odometry initializes
#while time() - timer <1.5:                                                      # 1.5 second delay.  This seems to improve odometry accuracy on reset
    #reset_odom.publish(Empty())
resetted = True
r = rospy.Rate(10)

#Main method
while not rospy.is_shutdown():
#obtain the x,y vector to goal
#    vel = sqrt(pow(delta.x,2)+pow(delta.y,2))
#use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(delta.y,delta.x) #tanx = O/A

#convert angle to degrees
    angle_to_goal = angle_to_goal
#find the difference between the angle of the bot and angle needed to turn
    angle =  angle_to_goal - current_th

#angle = (angle)%360
    if angle < (-pi):
        angle = angle + (2 * pi)
    if angle > pi:
        angle = angle - (2 * pi)
        print ("x: %s y: %s th: %s angle: %s" % (current_x, current_y, current_th, angle))
# 4.5 degree error is a comprimise between speed and accuracy
    if angle > 0.1:
        speed.angular.z = .5
    elif angle < -0.1:
        speed.angular.z = -.5
    elif angle < -1:
        speed.angular.z = -1.5
        speed.linear.x = 0
    elif angle > 1:
        speed.angular.z = 1.5
        speed.linear.x = 0
    else:
        speed.angular.z = 0
        speed.linear.x = 0.2
        if speed.linear.x < 0.1:
            speed.linear.x = 0.25
        if speed.linear.x > 0.2:
            speed.linear.x = 0.25
    if 0.25 >= abs(goal.x-current_x) and 0.25 >= abs(goal.y-current_y):
        speed.angular.z = 0
        speed.linear.x = 0
# check if the bot is within a suitable angle to the goal
    pub.publish(speed)
    r.sleep()
rospy.spin()
