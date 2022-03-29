#!/usr/bin/env python3
#=====================
import rospy
import numpy as np

from hardware_control.hw_topics import hw_topics
from hardware_control.msg import seeker_states

# Initialize classes
hw_top = hw_topics()

#==============================================================================
def seeker_control_node():
    rospy.init_node( "seeker_control_node", anonymous = True )
    rate = rospy.Rate( 1 )

    pub = rospy.Publisher( hw_top.seeker_states, seeker_states, queue_size = 1 )

    # Initialize seeker state topic
    skr_state = seeker_states()

    # Initialize seeker states
    skr_state.joint_angle.position = [ 0.0, 0.0 ]

    while not rospy.is_shutdown():
        # Don't move the seeker around for now
        skr_state.header.stamp = rospy.Time.now()
        pub.publish( skr_state )

#==============================================================================
if __name__ == "__main__":
    try:
        seeker_control_node()
    except rospy.ROSInterruptException:
        pass