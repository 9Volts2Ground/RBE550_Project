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
        self.seeker_gain = 0.07

        self.seeker_min = [0,0]
        self.seeker_max = [0,0]
        for servo in range( hrd.num_seeker_joints ):
            self.seeker_min[servo] = np.deg2rad( hrd.seekerMin[servo] - hrd.seekerCenter[servo] )
            self.seeker_max[servo] = np.deg2rad( hrd.seekerMax[servo] - hrd.seekerCenter[servo] )

        # Turn on publisher and subscribers
        self.pub = rospy.Publisher( hw_top.seeker_states, seeker_states, queue_size = 1 )
        self.pub.publish( self.skr_state ) # Initialize some states to grab

        rospy.Subscriber( w_top.target_track, target_track, self.seeker_control )

    #================================================================
    def seeker_control( self, target_track ):

        if 'target_track' in target_track.tracking_state:

            # We are tracking a target. See if we need to point the seeker az servo
            if abs( target_track.tgt_pos_in_frame.x ) > self.target_centered_tolerance:
                # The target is not centered in the frame. Point the seeker
                self.skr_state.joint_angle.position[0] += target_track.tgt_pos_in_frame.x * self.seeker_gain
                # Make sure seeker angles stay within bounds
                self.skr_state.joint_angle.position[0] = np.max( [ np.min( [self.skr_state.joint_angle.position[0], self.seeker_max[0] ] ), self.seeker_min[0] ] )

            # We are tracking a target. See if we need to point the seeker el servo
            if abs( target_track.tgt_pos_in_frame.y ) > self.target_centered_tolerance:
                # The target is not centered in the frame. Point the seeker
                self.skr_state.joint_angle.position[1] += target_track.tgt_pos_in_frame.y * self.seeker_gain
                # Make sure seeker angles stay within bounds
                self.skr_state.joint_angle.position[1] = np.max( [ np.min( [self.skr_state.joint_angle.position[1], self.seeker_max[1] ] ), self.seeker_min[1] ] )

            self.skr_state.header.stamp = rospy.Time.now()
            self.pub.publish( self.skr_state )

#==============================================================================
if __name__ == "__main__":
    try:
        sc = seeker_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass