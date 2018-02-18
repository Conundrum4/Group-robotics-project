#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, pow, sqrt
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan

#initialize values
current_x = 0.0                                                                 # current x co-ord of robot
current_y = 0.0                                                                 # current y co-ord of robot
current_th = 0.0                                                                # current orientation of the robot

turn = 0.0                                                                      # holds co-ords of next maneuver

achieved = True                                                                 # boolean - has the robot reached it's goal?
resetted = False                                                                # boolean - has the odometry been reset?

#Set the goal coordinates
goal = Point()                                                                  # co-ordinates of the final goal
goal.x = 6
goal.y = 2

sub_goal = Point()                                                              # a subgoal 0.5 meters from the robot
sub_goal.x = 0
sub_goal.y = 0

#set the vector increments for a 0.5m radius
def turn_options(index):                                                        # This function holds 7 possible turning options (windows)
    global current_x                                                            # which will be added to the current co-ordinates to determine
    global current_y                                                            # the location of the next subgoal 0 -> 7 , left -> right
    global turn
    turn = Point()
    x = 0
    y = 0

    # 75 degrees left
    if index == 0:
        x = current_x + 0.13
        y = current_y + 0.48
    # 50 degrees left
    if index == 1:
        x = current_x + 0.32
        y = current_y + 0.37
    # 25 degrees left
    if index == 2:
        x = current_x + 0.45
        y = current_y + 0.21
    # straight ahead
    if index == 3:
        x = current_x + 0.5
        y = current_y + 0
    # 25 degrees right
    if index == 4:
        x = current_x + 0.45
        y = current_y - 0.21
    # 50 degrees right
    if index == 5:
        x = current_x + 0.32
        y = current_y - 0.37
    # 75 degrees right
    if index == 6:
        x = current_x + 0.13
        y = current_y - 0.48

    turn.x = x
    turn.y = y
    return turn

#Obtain the shortest distance to the goal for a paticular set of co-ords
def dist_to_goal(x,y):                                                          # This function determines the distance between any two sets of
    global goal                                                                 # co-ordinates using pythagoras.
    vect = sqrt(pow((goal.x -x),2) + pow((goal.y-y),2))
    #print ("Distance to goal: %s" %(vect))
    return vect

#Is there an obstacle within 0.7m of the robot on a chosen path?
def check_ranges(distance):                                                     # This function checks if there is an object closer than 0.7 meters
    if distance < 0.7:                                                          # from the robot and returns True only if there are no obstacles
        return False                                                            # detected within this range
    return True

#Find the x and y co-ords of the goal
def find_angle(goal_x,goal_y, x, y):                                            # This functions determines the relative angle between two sets
    global goal                                                                 # of co-ordinates and is used to orient and turn the robot.
    inc_x = goal.x - x
    inc_y = goal.y - y
    #use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(inc_y, inc_x) #tanx = O/A
    #convert angle to degrees
    angle_to_goal = angle_to_goal*(180/pi)
    return angle_to_goal


#set the ranges attributed to each option
def steering(data):
    global sub_goal
    global achieved
    laser_ranges = [None]*7                                                     #an array to store the minimum range values in each window
    new_coords = [None]*7                                                       #an array to store potential new co-ords to move to
    no_obstruction = [None]*7                                                   #an array to store viable new co-ords (no obstruction present)
    closest = [None]*7                                                          #an array to store the distances of the new co-ords from the goal

    #In gazebo left to right(0 -> 6), on robot right to left (6 <- 0)
    six = data.ranges[608:719]
    five = data.ranges[509:608]
    four = data.ranges[409:508]
    three = data.ranges[313:408]
    two = data.ranges[213:312]
    one = data.ranges[113:212]
    zero = data.ranges[0:112]

    #an array of the ranges
    laser_ranges = [zero, one, two, three, four, five, six]                     # An array to hold the range values

    i = 0                                                                       # index for for loop
    j = 0                                                                       # index for for loop

    if(achieved == False):                                                      # Check if you have reached the goal.  If not keep going
        #print 'I am not there yet!'
	    return

    if(resetted == False):                                                      # Checks that the odometry has been reset
	    return

    for i in range(7):
        new_coords[i] = turn_options(i)                                         # Adds the new co-ords to the array
        closest[i] = dist_to_goal(turn_options(i).x, turn_options(i).y)         # Adds distance to goal

        if min(laser_ranges[i]) > 0.7:                                          # Checks if there is an obstruction
            no_obstruction[i] = 1                                               # This is a viable option
        else:
            no_obstruction[i]= 0                                                # There is an obstruction, not a viable option
            closest[i] = 20*closest[i]                                          # *20 to make sure that obstructed co-ords are not seen as closest
            print no_obstruction
	        print closest
                                                                                # There is an obstruction present

    for j in range(7):
        if (no_obstruction[j] == 1) and (closest[j] == min(closest)):           # Checks laser ranges and dist to goal
            sub_goal = new_coords[j]                                            # Updates the subgoal with co-ords that have no obstructions
            print 'goals'                                                       # and is the closest viable option to the goal
            print sub_goal.x
            print sub_goal.y
            achieved = False                                                    # Updates the achieved boolean
        else:
            print 'nowhere to go!'


#Odometry callback
def newOdom(msg):
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
sub = rospy.Subscriber("/odom", Odometry, newOdom)                              # Odometry subscriber
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
    inc_x = sub_goal.x - current_x
    inc_y = sub_goal.y - current_y
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
