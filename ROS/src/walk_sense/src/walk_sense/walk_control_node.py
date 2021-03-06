#!/usr/bin/env python3
#=====================
import copy
import rospy
import numpy as np
import time
from geometry_msgs.msg import TwistStamped

from hardware_control.hw_topics import hw_topics
from hardware_control.msg import seeker_states
from sensor_msgs.msg import Range # Standard ROS message type
from walk_sense.msg import target_states
from walk_sense.msg import target_track
from walk_topics import walk_topics

hw_top = hw_topics()
w_top = walk_topics()

#==============================================================================
class walk_control_node():
    def __init__(self):
        rospy.init_node( "walk_control_node", anonymous = True )

        # Initialize topic data to work with
        self.target_states = target_states()
        self.range_sensor = Range()
        self.range_sensor.range = -np.inf # Init to no return
        self.seeker_states = seeker_states()
        self.seeker_states.joint_angle.position = [0.0, 0.0]

        #----------------------------------
        # Define local variables. Detection states
        self.target_search_mode_options = ['target_search', 'target_track', 'target_lost', 'target_acquired']
        self.target_search_mode = self.target_search_mode_options[0]
        self.previous_target_search_mode = self.target_search_mode
        self.azimuth_position = 0.0
        self.elevation_position = 0.0
        self.target_lost_time = time.time()
        self.target_lost_timeout = 2.0
        self.target_acquire_distance = 0.5 # meters

        # Other logic variables
        self.angular_moment = 0.15 # Approximate distance from body to foot
        self.max_velocity = 0.03 # m/s
        self.target_centered_tolerance = 0.1 # Acceptable distance from center of image, % of frame
        self.seeker_turned_tolerance = 0.436 # ~25 degrees off center threshold to add spin to the robot
        self.seeker_az_gain = 0.01
        self.target_az_side = 1 # >0 = left, <0 = right
        self.spin_gain = 0.05 # Scales error relative to image position into angular velocity

        # Turn on publisher and subscribers
        rospy.Subscriber( w_top.target_states, target_states, self.grab_target_states )
        rospy.Subscriber( w_top.range_sensor, Range, self.grab_range_states )
        rospy.Subscriber( hw_top.seeker_states, seeker_states, self.grab_seeker_states )
        self.pub = rospy.Publisher( w_top.walk_twist, TwistStamped, queue_size = 1 )
        self.pub_track = rospy.Publisher( w_top.target_track, target_track, queue_size = 1 )

        self.walk_control_node()

    #======================================================
    def walk_control_node( self ):
        rate = rospy.Rate( 10 )

        print("Entering walking while loop...")

        while not rospy.is_shutdown():

            # Grab static, local copies of states to work with
            target_state = copy.deepcopy( self.target_states )
            range_sensor = copy.deepcopy( self.range_sensor )

            #------------------------------------------------------------------------
            # Determine which state we need to be in
            if self.previous_target_search_mode == self.target_search_mode_options[3]:
                # Stay acquired
                self.target_search_mode = self.target_search_mode_options[3]
            elif target_state.target_found:
                # Tracking target
                self.target_search_mode = self.target_search_mode_options[1]
                if range_sensor.range <= self.target_acquire_distance and range_sensor.range >= range_sensor.min_range:
                    # If we get close enough, stop moving
                    self.target_search_mode = self.target_search_mode_options[3]
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

            print(f"Moving to {self.target_search_mode} mode. Range = {range_sensor.range}")

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

            # Package target search mode history again
            self.previous_target_search_mode = self.target_search_mode

            rate.sleep()

    #======================================================
    def target_search_logic( self ):
        """
        Spin the robot until it finds the target.
        Start spinning counter-clockwise (+z), until we have lost a target,
        then spin the direction we last saw a target
        """
        twist = self.init_twist()
        twist.twist.angular.z = self.max_velocity / self.angular_moment * self.target_az_side # target_az_side sets direction (+/-z)
        return twist

    #======================================================
    def target_track_logic( self, target_state ):
        '''
        Move the robot in the direction of the identified target
        '''
        twist = self.init_twist()

        twist.twist.linear.x = self.max_velocity # Command it to walk forward towards the target

        # Spin until we center the target in frame
        half_image_width = target_state.camera_width / 2
        half_image_height = target_state.camera_height / 2
        self.azimuth_position = ( half_image_width - target_state.target_position.x ) / half_image_width # Left half of image is positive
        self.elevation_position = ( half_image_height - target_state.target_position.y ) / half_image_height
        self.target_az_side = np.sign( self.azimuth_position ) # Track which side the target was last seen on

        # If target is not centered in frame, add some spin to the gait
        if abs( self.azimuth_position ) > self.target_centered_tolerance:
            twist.twist.angular.z += self.azimuth_position * self.spin_gain

        # If the seeker isn't pointing straight forward, spin to try and center the seeker
        if abs( self.seeker_states.joint_angle.position[0] ) > self.seeker_turned_tolerance:
            twist.twist.angular.z += self.seeker_states.joint_angle.position[0] * self.seeker_az_gain

        twist = self.scale_twist( twist )

        return twist

    #======================================================
    def target_lost_logic( self ):
        '''
        Move in the same direction you last saw the target
        '''
        twist = self.init_twist()
        twist.twist.linear.x = self.max_velocity # Command it to walk forward, hoping the target is still there
        twist.twist.angular.z = 0.2 * self.target_az_side # rad/s
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
    def grab_range_states( self, range_sensor ):
        """
        Grabs range_sensor data from ultrasonic range sensor topic to store locally
        """
        self.range_sensor = copy.deepcopy( range_sensor )

    #======================================================
    def grab_seeker_states( self, seeker_states ):
        """
        Grabs seeker_states data from seeker servo controller topic to store locally
        """
        self.seeker_states = copy.deepcopy( seeker_states )

    #======================================================
    def scale_twist( self, twist ):
        '''
        Makes sure the twist magnitude is not too large
        '''
        linear_velocity_mag = np.linalg.norm( [ twist.twist.linear.x,
                                                twist.twist.linear.y,
                                                twist.twist.linear.z] )
        angular_velocity_mag = np.linalg.norm( [twist.twist.angular.x,
                                                twist.twist.angular.y,
                                                twist.twist.angular.z] ) * self.angular_moment
        scaled_velocity_mag = self.max_velocity / ( linear_velocity_mag + angular_velocity_mag )

        twist.twist.linear.x *= scaled_velocity_mag
        twist.twist.linear.y *= scaled_velocity_mag
        twist.twist.linear.z *= scaled_velocity_mag
        twist.twist.angular.x *= scaled_velocity_mag
        twist.twist.angular.y *= scaled_velocity_mag
        twist.twist.angular.z *= scaled_velocity_mag

        return twist

    #======================================================
    def init_twist( self ):
        '''
        Initialize twist vector
        '''
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        return twist

#==============================================================================
if __name__ == "__main__":
    try:
        wc = walk_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
