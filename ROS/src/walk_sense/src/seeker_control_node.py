#!/usr/bin/env python3
#=====================
import rospy
import numpy as np

from classes import walk_topics
from hardware_control import hardware_constants
from hardware_control.hw_topics import hw_topics
from hardware_control.msg import seeker_states
from walk_sense.msg import target_track

# Initialize classes
hrd = hardware_constants.hardware_constants()
hw_top = hw_topics()
w_top = walk_topics.walk_topics()

#==============================================================================
class seeker_control_node():
    def __init__(self):
        rospy.init_node( "seeker_control_node", anonymous = True )

        # Define class variables
        self.skr_state = seeker_states() #  Initialize seeker state topic
        self.skr_state.joint_angle.position = [ 0.0, 0.0 ]

        # Gain values to tune
        self.target_centered_tolerance = 0.1
        self.el_gain = 0.2
        self.el_min = np.deg2rad( hrd.seekerMin[1] - hrd.seekerCenter[1] )
        self.el_max = np.deg2rad( hrd.seekerMax[1] - hrd.seekerCenter[1] )

        # Turn on publisher and subscribers
        self.pub = rospy.Publisher( hw_top.seeker_states, seeker_states, queue_size = 1 )
        rospy.Subscriber( w_top.target_track, target_track, self.seeker_el_control )

    #================================================================
    def seeker_el_control( self, target_track ):

        if 'target_track' in target_track.tracking_state:
            # We are tracking a target. See if we need to point the seeker el servo
            if abs( target_track.tgt_pos_in_frame.y ) > self.target_centered_tolerance:
                # The target is not centered in the frame. Point the seeker
                self.skr_state.joint_angle.position[1] += target_track.tgt_pos_in_frame.y * self.el_gain

                # Make sure seeker angles stay within bounds
                self.skr_state.joint_angle.position[1] = np.max( [ np.min( [self.skr_state.joint_angle.position[1], self.el_max ] ), self.el_min ] )


        self.skr_state.header.stamp = rospy.Time.now()
        self.pub.publish( self.skr_state )

#==============================================================================
if __name__ == "__main__":
    try:
        sc = seeker_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass