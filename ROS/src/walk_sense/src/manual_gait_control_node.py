#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

# Standard ROS libraries
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

# Custom libraries
from classes import walk_topics
from hardware_control.hw_topics import hw_topics
from hardware_control.msg import seeker_states

hw_top = hw_topics()
w_top = walk_topics.walk_topics()

#==============================================================================
class joy_stick():
    def __init__(self):
        ''' Define directions the joystick can go'''
        self.fwd_back = 0.0
        self.right_left = 0.0
        self.down = 0 # Center click joy stick

#==============================================================================
class xbox_controller():
    def __init__(self):
        self.l_stick = joy_stick()
        self.r_stick = joy_stick()
        self.d_pad = joy_stick()
        self.r_trigger = 0.0
        self.l_trigger = 0.0

        # Buttons
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0
        self.r_shoulder = 0
        self.l_shoulder = 0
        self.center = 0
        self.back = 0 # Left control button
        self.home = 0 # Right control button

#==============================================================================
class manual_gait_control_node():
    def __init__(self):
        rospy.init_node( "manual_gait_control_node", anonymous=True)

        # Initialize xbox controller instance for parsing
        self.xbox = xbox_controller()

        # Initialize body pose state
        self.pose = TransformStamped()

        # Initialize gait twist state
        self.twist = TwistStamped()
        self.clear_twist()

        # Define gait parameters/gains
        self.max_velocity = 0.05 # m/s
        self.max_angular_velocity = np.pi/10 # rad/s

        self.max_body_height = 0.1 # m
        self.pose_height_rate = 0.01 # m/s

        # Time stamp, used to integrate positions
        self.t = rospy.get_time()
        self.t_previous = rospy.get_time()
        self.dt = 0.0

        # Initialize seeker states
        self.skr_state = seeker_states() #  Initialize seeker state topic
        self.skr_state.joint_angle.position = [ 0.0, 0.0 ]

        # Define publishers
        self.pub_pose = rospy.Publisher( w_top.body_ground_transform, TransformStamped, queue_size=1 )
        self.pub_twist = rospy.Publisher( w_top.walk_twist, TwistStamped, queue_size = 1 )
        self.pub_skr = rospy.Publisher( hw_top.seeker_states, seeker_states, queue_size = 1 )

        # Initialize subscription to joy controller node
        rospy.Subscriber( "joy", Joy, self.manual_control )

    #======================================================
    def manual_control( self, joy ):

        # Parse Joy inputs
        self.map_joy_to_xbox( joy )

        # Calculate dt since last change
        self.t = rospy.get_time()
        self.dt = self.t - self.t_previous

        # ToDo: Calculate commanded body pose
        self.calculate_body_pose()

        # Calculate comanded walking gait twist
        self.calculate_gait_twist()

        # ToDo: Calculate commanded seeker angles
        self.calculate_seeker_states()

        self.t_previous = self.t

    #======================================================
    def map_joy_to_xbox( self, joy ):
        ''' Unpackage the /joy topic to something useful'''
        self.xbox.l_stick.fwd_back = joy.axes[1]
        self.xbox.l_stick.right_left = -joy.axes[0]
        self.xbox.l_stick.down = joy.buttons[9]

        self.xbox.r_stick.fwd_back = joy.axes[4]
        self.xbox.r_stick.right_left = -joy.axes[3]
        self.xbox.r_stick.down = joy.buttons[10]

        self.xbox.d_pad.fwd_back = joy.axes[7]
        self.xbox.d_pad.right_left = -joy.axes[6]

        self.xbox.r_trigger = joy.axes[5]
        self.xbox.l_trigger = joy.axes[2]

        self.xbox.a = joy.buttons[0]
        self.xbox.b = joy.buttons[1]
        self.xbox.x = joy.buttons[2]
        self.xbox.y = joy.buttons[3]

        self.xbox.r_shoulder = joy.buttons[5]
        self.xbox.l_shoulder = joy.buttons[4]

        self.xbox.center = joy.buttons[8]
        self.xbox.back = joy.buttons[6]
        self.xbox.home = joy.buttons[7]

    #======================================================
    def calculate_body_pose( self ):
        ''' Convert /joy topic into commanded body pose'''

        self.pose.header.stamp = rospy.Time.now()

        # D-pad controls body height
        self.pose.transform.translation.z += self.xbox.d_pad.fwd_back * self.dt * self.pose_height_rate

        # Keep the body pose reasonable
        self.pose.transform.translation.z = np.max( [ np.min( [ self.pose.transform.translation.z,
                                                               self.max_body_height ] ), 0.0 ] )

        self.pub_pose.publish( self.pose )

    #======================================================
    def calculate_gait_twist( self ):
        ''' Convert /joy topic into commanded twist vector'''

        # Calculate linear vector from left joystick
        scaled_linear_vec = np.array( [ self.xbox.l_stick.fwd_back,
                                       -self.xbox.l_stick.right_left,
                                        0.0 ] ) * self.max_velocity # No vertical velocity vector. We can't fly yet :/
        self.twist.twist.linear.x = scaled_linear_vec[0]
        self.twist.twist.linear.y = scaled_linear_vec[1]

        # Calculate angular vector from right joystick
        # Note: positive Z is counter-clockwise, hence - right_left
        self.twist.twist.angular.z = -self.xbox.r_stick.right_left * self.max_angular_velocity

        # Send out desired walking gait twist vector
        self.twist.header.stamp = rospy.Time.now()
        self.pub_twist.publish( self.twist )

    #======================================================
    def calculate_seeker_states( self ):
        ''' Convert /joy topic into commanded seeker states'''
        self.pub_skr.publish( self.skr_state )

    #======================================================
    def clear_twist( self ):
        ''' Clears out commanded twist vector '''
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = 0.0
        self.twist.twist.linear.y = 0.0
        self.twist.twist.linear.z = 0.0
        self.twist.twist.angular.x = 0.0
        self.twist.twist.angular.y = 0.0
        self.twist.twist.angular.z = 0.0

#==============================================================================
if __name__ == "__main__":
    try:
        mc = manual_gait_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
