#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

rospy.init_node('publisher')

pub = rospy.Publisher('/message', String, queue_size=10)

message = rospy.get_param('~message', 'Hello World Default!')
hz      = rospy.get_param('~hz', 2)
 
rate = rospy.Rate(hz)


while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()
    
