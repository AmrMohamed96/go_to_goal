#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Byte
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import math

# i is the number of target points along the path
i = 0

# defining ros publishers
cmd_vel_publisher = rospy.Publisher('cmd_vel_rob1', Twist, queue_size = 5)
gtg_flag_publisher = rospy.Publisher('gtg_flag_rob1', Byte , queue_size = 5)

# robot current position variables
x = 0
y = 0
theta = 0

# robot target position
x_y_goal_list = []

# dimenstion of each square in the grid
grid_size = 17.5

# encoder delay function variables
req_dist_x = 0
req_dist_y = 0
approach_distance = 5   # set this as you wish  

current_enc_x = 0       # current encoder distance recorded
current_enc_y = 0       # current encoder distance recorded
screenshot_x = 0        # the value of x to compare to
screenshot_y = 0        # the value of y to compare to

prev_x_g = 0
prev_y_g = 0

def robot_current_pos_callback(pos):
    global x, y, theta
    x = pos.data[0]
    y = pos.data[1]
    theta = pos.data[2]/100.0 #in rad

def robot_goal_callback(head):
    global x_y_goal_list
    x_y_goal_list = head.data

def encoder_position_callback(data):
    global screenshot_x, screenshot_y
    current_enc_x = data.data[0]
    current_enc_y = data.data[1]

def gtg_talker():
    # initializing the ROS node
    rospy.init_node('go_to_goal')
    rospy.loginfo("%s started" % rospy.get_name())

    # initialize ROS Subscribers
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, robot_current_pos_callback)
    rospy.Subscriber('Planning_Output1', Int32MultiArray, robot_goal_callback) 
    rospy.Subscriber('CurrentPosition_rob1', Int32MultiArray, encoder_position_callback) 

    # defining the publishing rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        get_twist(x, y, theta, x_y_goal_list)
        rate.sleep()

def update_encoders_goal():
    global screenshot_x, screenshot_y, current_enc_x, current_enc_y, req_dist_x, req_dist_y
    # saving the current values to compare to
    screenshot_x = current_enc_x
    screenshot_y = current_enc_y

    # calculating distance it should cover
    req_dist_x = abs( x_y_goal_list[0] - x)
    req_dist_y = abs( x_y_goal_list[1] - y)

def check_encoders_approach():
    approaching_error_x = abs( current_enc_x - screenshot_x)
    approaching_error_y = abs( current_enc_y - screenshot_y)

    if (approaching_error_x <= approach_distance) or (approaching_error_y <= approach_distance):
        # sleep for 0.5 seconds if the target is close
        rospy.sleep(0.5)

def get_twist(x, y, theta, xy_g):
    global i, prev_x_g, prev_y_g

    # for the first run, get the number of goals and store it in i
    if not i:
        i = len(xy_g)

    # if the length has been fetched, start the iterations of the go to goal on each point
    if i>0:
        # reset this flag because it is used to keep the gtg inside the loop if it hasn't reached the goal yet
        flag=0

        if (i%2 == 0):
            i-=1

        x_g = ((xy_g[i])-1)* grid_size
        y_g = ((xy_g[i-1])-1)* grid_size

        # this function will update the goal for the encoder to make it's checks
        if (prev_x_g != x_g) and (prev_y_g != y_g):
                    update_encoders_goal()
        prev_x_g = x_g 
        prev_y_g = y_g 
        
        # this is the main GTG loop
        if not flag:
            # this function will cause a delay if the robot approaches the target
            check_encoders_approach()

            # get the error in the global reference frame
            error_x  = x_g  - x
            error_y  = y_g  - y

            if((error_x>=-3) and (error_x<=3)):
                error_x = 0
            if((error_y>=-3) and (error_y<=3)):
                error_y = 0

            # get the error in the robot's reference frame
            # @Xreference//Ycam
            gr_y  = error_x*math.cos(theta)  + error_y*math.sin(theta) #takes rad
            gr_x  = -error_x*math.sin(theta) + error_y*math.cos(theta)

            # calculate rho and alfa
            rho  = math.sqrt(gr_x**2+gr_y**2) #in cm
            alfa = math.atan2(gr_y,gr_x) #in rad

            # define controller gains
            K_RHO = 1.4
            K_ALPHA = 0.5
            
            #calculate control commands
            v     = K_RHO*rho       # in cm/sec             # v = linear velocity command
            omega = K_ALPHA*alfa    # in rad/sec            # omega = angular velocity command
                  
            # return the control commands
            twist = Twist()

            if ((omega<=0.2) and (omega>=-0.2)) :
                omega = 0

            if omega:
                twist.linear.x  = 0
                twist.linear.y  = 0 
                twist.linear.z  = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = omega

            elif not omega:
                twist.linear.x  = v
                twist.linear.y  = 0
                twist.linear.z  = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0

        cmd_vel_publisher.publish(twist)

        if(gr_x==0 and gr_y==0):
            flag=1

        if flag:
            i-=2

    # this part executes when the go to goal have finished all points   
    if i<0:
        twist = Twist()
        gtg_flag = 1
        twist.linear.x  = 0
        twist.linear.y  = 0 
        twist.linear.z  = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = omega

        # published the stop command and publish the raised flag
        cmd_vel_publisher.publish(twist)
        gtg_flag_publisher.publish(gtg_flag)

if __name__ == '__main__':
    try:
        gtg_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed" % rospy.get_name())
