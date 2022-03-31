#!/usr/bin/env python3
#=====================
import copy
import rospy
import numpy as np
import time

from classes import walk_topics
from walk_sense.msg import walk_twist
from walk_sense.msg import target_states
from walk_sense.msg import target_track

w_top = walk_topics.walk_topics()

#==============================================================================
class walk_control_node():
    def __init__(self):
        rospy.init_node( "walk_control_node", anonymous = True )

        # Initialize topic data to work with
        self.target_states = target_states()
        # self.range_states = range_states()

        #----------------------------------
        # Define local variables. Detection states
        self.target_search_mode_options = ['target_search', 'target_track', 'target_lost', 'target_acquired']
        self.target_search_mode = self.target_search_mode_options[0]
        self.previous_target_search_mode = self.target_search_mode
        self.azimuth_position = 0.0
        self.elevation_position = 0.0
        self.target_lost_time = time.time()
        self.target_lost_timeout = 2.0
        self.target_acquire_distance = 1.5 # meters

        # Other logic variables
        self.angular_moment = 0.05 # Approximate distance from body to foot
        self.max_velocity = 0.015 # m/s
        self.target_centered_tolerance = 0.1 # Acceptable distance from center of image
        self.target_az_side = 1 # >0 = left, <0 = right
        self.spin_gain = 0.25 # Scales error relative to image position into angular velocity

        # Turn on publisher and subscribers
        rospy.Subscriber( w_top.target_states, target_states, self.grab_target_states )
        # rospy.Subscriber( w_top.range_states, range_states, self.grab_range_states )
        self.pub = rospy.Publisher( w_top.walk_twist, walk_twist, queue_size = 1 )
        self.pub_track = rospy.Publisher( w_top.target_track, target_track, queue_size = 1 )

        self.walk_control_node()

    #======================================================
    def walk_control_node( self ):
        rate = rospy.Rate( 10 )

        print("Entering walking while loop...")

        while not rospy.is_shutdown():

            # Grab static, local copies of states to work with
            target_state = copy.deepcopy( self.target_states )
            # range_state = copy.deepcopy( self.range_states )

            #------------------------------------------------------------------------
            # Determine which state we need to be in
            if self.previous_target_search_mode == self.target_search_mode_options[3]:
                # Stay acquired
                self.target_search_mode = self.target_search_mode_options[3]
            elif target_state.target_found:
                # Tracking target
                self.target_search_mode = self.target_search_mode_options[1]
                # if range_state.range <= self.target_acquire_distance:
                #     self.target_search_mode = self.target_search_mode_options[3]
            elif self.previous_target_search_mode == self.target_search_mode_options[1]:
                # First frame target lost
                self.target_search_mode = self.target_search_mode_options[2]
                self.target_lost_time = time.time()

            elif self.previous_target_search_mode == self.target_search_mode_options[2] and \
                (time.time() - self.target_lost_time >= self.target_lost_timeout):
                # We've lost the target for long enough, go back to target search
                self.target_search_mode = self.target_search_mode_options[0]
            else:
                # Maintain current state
                self.target_search_mode = self.previous_target_search_mode

            #------------------------------------------------------------------------
            # Do state logic
            if self.target_search_mode == self.target_search_mode_options[0]:
                twist = self.target_search_logic()
            elif self.target_search_mode == self.target_search_mode_options[1]:
                twist = self.target_track_logic( target_state )
            elif self.target_search_mode == self.target_search_mode_options[2]:
                twist = self.target_lost_logic()
            else: # Target already acquired
                twist = self.target_found_logic()

            # Package target search mode history again
            self.previous_target_search_mode = self.target_search_mode

            # Send out the commanded twist
            self.pub.publish( twist )

            # Send out the target track states
            track = target_track()
            track.header.stamp = rospy.Time.now()
            track.tracking_state = self.target_search_mode
            track.tracking_state_prev = self.previous_target_search_mode
            track.tgt_pos_in_frame.x = self.azimuth_position
            track.tgt_pos_in_frame.y = self.elevation_position
            self.pub_track.publish( track )

            rate.sleep()

    #======================================================
    def target_search_logic( self ):
        """
        Spin the robot until it finds the target
        """
        twist = self.init_twist()
        twist.walk_direction.angular.z = self.max_velocity / self.angular_moment * self.target_az_side
        return twist

    #======================================================
    def target_track_logic( self, target_state ):
        '''
        Move the robot in the direction of the identified target
        '''
        twist = self.init_twist()

        twist.walk_direction.linear.y = self.max_velocity # Command it to walk forward towards the target

        # Spin until we center the target in frame
        half_image_width = target_state.camera_width / 2
        half_image_height = target_state.camera_height / 2
        self.azimuth_position = ( half_image_width - target_state.target_position.x ) / half_image_width # Lef half of image is positive
        self.elevation_position = ( half_image_height - target_state.target_position.y ) / half_image_height
        self.target_az_side = np.sign( self.azimuth_position ) # Track which side the target was last seen on

        if abs( self.azimuth_position ) > self.target_centered_tolerance:
            # Target is not centered in frame. Add some spin to the gait
            twist.walk_direction.angular.z = self.azimuth_position * self.spin_gain
            twist = self.scale_twist( twist )

        # Adjust the seeker

        # Check range sensor for termination

        return twist

    #======================================================
    def target_lost_logic( self ):
        '''
        Move in the same direction you last saw the target
        '''
        twist = self.init_twist()
        twist.walk_direction.linear.y = self.max_velocity # Command it to walk forward, hoping the target is still there
        twist.walk_direction.angular.z = 0.2 * self.target_az_side # rad/s
        twist = self.scale_twist( twist )
        return twist

    #======================================================
    def target_found_logic( self ):
        '''
        When the target has been acquired, stop moving
        '''
        return self.init_twist()

    #======================================================
    def grab_target_states( self, target_states ):
        """
        Grabs target_states data from image processing topic to store locally
        """
        self.target_states = copy.deepcopy( target_states )

    #======================================================
    def grab_range_states( self, range_states ):
        """
        Grabs range_states data from ultrasonic range sensor topic to store locally
        """
        self.range_states = copy.deepcopy( range_states )

    #======================================================
    def scale_twist( self, twist ):
        '''
        Makes sure the twist magnitude is not too large
        '''
        linear_velocity_mag = np.linalg.norm( [ twist.walk_direction.linear.x,
                                                twist.walk_direction.linear.y,
                                                twist.walk_direction.linear.z] )
        angular_velocity_mag = np.linalg.norm( [twist.walk_direction.angular.x,
                                                twist.walk_direction.angular.y,
                                                twist.walk_direction.angular.z] ) * self.angular_moment
        scaled_velocity_mag = self.max_velocity / ( linear_velocity_mag + angular_velocity_mag )

        twist.walk_direction.linear.x *= scaled_velocity_mag
        twist.walk_direction.linear.y *= scaled_velocity_mag
        twist.walk_direction.linear.z *= scaled_velocity_mag
        twist.walk_direction.angular.x *= scaled_velocity_mag
        twist.walk_direction.angular.y *= scaled_velocity_mag
        twist.walk_direction.angular.z *= scaled_velocity_mag

        return twist

    #======================================================
    def init_twist( self ):
        '''
        Initialize twist vector
        '''
        twist = walk_twist()
        twist.header.stamp = rospy.Time.now()
        twist.walk_direction.linear.x = 0.0
        twist.walk_direction.linear.y = 0.0
        twist.walk_direction.linear.z = 0.0
        twist.walk_direction.angular.x = 0.0
        twist.walk_direction.angular.y = 0.0
        twist.walk_direction.angular.z = 0.0
        return twist

#==============================================================================
if __name__ == "__main__":
    try:
        wc = walk_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass