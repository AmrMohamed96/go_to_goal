#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Int32MultiArray, Byte
from geometry_msgs.msg import Twist
from math import pi
import math
import numpy as np

# grid dimension parameters
grid_dimension = 0

# robot tag identifier
rob_id = -1

# robot position global vars
rob_x = 0
rob_y = 0
rob_theta = 0

# global flags
gtg_finish_flag = 0
intermediate_flag = 0

# goal points list
goal_points_flat = []
goal_points_lists = []

#################################################################
# CALLBACK FUNCTIONS
#################################################################
def rob_position_callback(data):
    global rob_x, rob_y, rob_theta
    rob_x = data.data[0]
    rob_y = data.data[1]
    rob_theta = data.data[2] /100

def goal_point_callback(data):
    global goal_points_flat, goal_points_lists
    goal_points_flat = data.data

    if len(goal_points_flat) > 0:
        goal_points_lists = np.reshape(goal_points_flat, (len(goal_points_flat)/2,2) )
        gtg_routine()

#################################################################
# MAIN TWIST CALCULATION
#################################################################
def get_twist(goal_point):
    global intermediate_flag
    x_g = goal_point[0] * grid_dimension
    y_g = goal_point[1] * grid_dimension

    print x_g, y_g, rob_x, rob_y, rob_theta
    # get the error in the global reference frame
    error_x  = x_g  - rob_x
    error_y  = y_g  - rob_y

    # thresholding the error
    if((error_x>=-5) and (error_x<=5)):
        error_x = 0
    if((error_y>=-5) and (error_y<=5)):
        error_y = 0

    # get the error in the robot's reference frame
    # @Xreference//Ycam
    gr_y  = error_x * math.cos(rob_theta)  + error_y * math.sin(rob_theta) #takes rad
    gr_x  = -error_x * math.sin(rob_theta) + error_y * math.cos(rob_theta)

    if gr_x == 0 and gr_y == 0:
        # an empty twist message to stop the robot
        twist = Twist()
        twist.linear.x  = 0; twist.linear.y  = 0; twist.linear.z  = 0;        
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0; 

        # raise the intermediate flag to fetch another point
        intermediate_flag = 1

        # return the empty twist message to be published
        twistPublisher.publish(twist)
        return

    # calculate rho and alfa
    rho  = math.sqrt(gr_x**2+gr_y**2) #in cm
    alfa = math.atan2(gr_y,gr_x) #in rad

    # define controller gains
    K_RHO = 1.4 # 0.6 doesn't work fine with me
    #(robot becomes unstable a bit after arriving to the desired pose)
    K_ALPHA = 0.5 #K_THETA = 0.5

    #calculate control commands
    v     = K_RHO*rho       # in cm/sec             # v = linear velocity command
    omega = K_ALPHA*alfa    # in rad/sec            # omega = angular velocity command

    # return th control commands
    twist = Twist()

    if((omega<=0.2) and (omega>=-0.2)):
        omega = 0

    if (omega!=0):
        twist.linear.x  = 0; twist.linear.y  = 0; twist.linear.z  = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = omega;

    elif (omega == 0):
        twist.linear.x  = v; twist.linear.y  = 0; twist.linear.z  = 0;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;

    twistPublisher.publish(twist)

#################################################################
# GTG Routine
#################################################################
def gtg_routine():
    global intermediate_flag, gtg_finish_flag, goal_points_lists, rob_x, rob_y, rob_theta

    if not gtg_finish_flag:
        if goal_points_lists.all():
            for i in range( len(goal_points_lists) ):
                intermediate_flag = 0
                while not intermediate_flag:
                    get_twist( goal_points_lists[i] )

            gtg_finish_flag = 1
            finalFlagPublisher.publish(gtg_finish_flag)
    else:
        pass


if __name__ == '__main__':
    # initialize the ros node
    rospy.init_node('gtg_rob1')
    rospy.loginfo('%s started' % rospy.get_name())

    grid_dimension = rospy.get_param('/grid_dimension', 25)
    rob_id = rospy.get_param('~rob_id', 1)

    # initialize the subscribers
    rospy.Subscriber('rob'+str(rob_id)+'_CurrentPose', Int32MultiArray, rob_position_callback)
    rospy.Subscriber('Planning_Output'+str(rob_id), Int32MultiArray, goal_point_callback)

    # initalize the publishers
    twistPublisher = rospy.Publisher('cmd_vel_rob'+str(rob_id), Twist, queue_size = 5)
    finalFlagPublisher = rospy.Publisher('gtg_flag_rob'+str(rob_id), Int32 , queue_size = 5)

    # initalize publishing rate
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.spin()