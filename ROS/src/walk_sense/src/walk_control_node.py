#!/usr/bin/env python3
#=====================
import rospy

import numpy as np

from classes import walk_topics
from walk_sense.msg import walk_twist

w_top = walk_topics.walk_topics()

#==============================================================================
def walk_control_node():
    rospy.init_node( "walk_control_node", anonymous = True )
    rate = rospy.Rate( 50 )

    pub = rospy.Publisher( w_top.walk_twist, walk_twist, queue_size = 1 )

    # Initialize twist
    twist = walk_twist()
    twist.walk_direction.linear.x = 0.000
    twist.walk_direction.linear.y = 0.05
    twist.walk_direction.linear.z = 0
    twist.walk_direction.angular.x = 0
    twist.walk_direction.angular.y = 0
    twist.walk_direction.angular.z = 0 #np.pi/50

    print("Entering walking while loop...")

    while not rospy.is_shutdown():

        pub.publish( twist )

        rate.sleep()

#==============================================================================
if __name__ == "__main__":
    try:
        walk_control_node()
    except rospy.ROSInterruptException:
        pass