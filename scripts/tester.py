#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Int32MultiArray
import time

rospy.init_node('gtg_test')
goals = [1,1,2,2,3,3]

pub = rospy.Publisher('Planning_Output1', Int32MultiArray, latch=True, queue_size=1)
time.sleep(3)

pub.publish( Int32MultiArray(data=goals) )

print "DONE"