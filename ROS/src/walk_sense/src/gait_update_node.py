#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

# Custom libraries and class instances
from classes.gait import gait
from classes.walk_topics import walk_topics
from functions.foot_position_to_joint_angles import *
from walk_sense.msg import walk_twist

from hardware_control.hw_topics import hw_topics # List of acceptable channel names
from hardware_control import hardware_constants
from hardware_control import rotation
from hardware_control.msg import leg_states # Custom ROS message types

# Initialize classes
hw_top = hw_topics()
w_top = walk_topics()
gt = gait()
hrd = hardware_constants.hardware_constants()

#==============================================================================
class gait_update_node():
    def __init__(self):

        # Initialize ROS communication
        rospy.init_node( "gait_update_node", anonymous=True )

        # Initialize local vars
        self.t_start = rospy.get_time()
        self.t_previous = self.t_start
        self.dt = 0
        self.phase = 0

        # Stored vars for up-phase
        self.last_lifting_phase  = [1,1,1,1,1,1]
        self.last_dropping_phase = [1,1,1,1,1,1]
        self.up_phase_foot_limit = np.zeros( (hrd.num_joints,hrd.num_legs) )
        self.distance_foot_traveled = np.zeros( hrd.num_legs )

        # Initialze foot position
        joint_ang = foot_position_to_joint_angles( gt.foot_center )

        # Publish a state of each leg to its own topic
        self.lg_st_msg = []
        self.pub = []
        for leg in range( hrd.num_legs ):
            self.pub.append( rospy.Publisher( hw_top.leg_states[leg], leg_states, queue_size = 1 ) )
            self.lg_st_msg.append( leg_states() )
            self.lg_st_msg[leg].leg_num = leg
            # Initialize foot states
            self.lg_st_msg[leg].foot_position.x = gt.foot_center[0,leg]
            self.lg_st_msg[leg].foot_position.y = gt.foot_center[1,leg]
            self.lg_st_msg[leg].foot_position.z = gt.foot_center[2,leg]
            self.lg_st_msg[leg].joint_angle.position = joint_ang[:,leg]

        # Subscribe to walk control after initializing all your stuff
        rospy.Subscriber( w_top.walk_twist, walk_twist, self.update_gait )

    #==============================================================================
    def update_gait( self, walk_twist ):

        t = rospy.get_time()
        self.dt = t - self.t_previous

        # Find what part of the gait phase we are in
        gt.phase += np.max( self.distance_foot_traveled) / gt.max_stride_length
        while ( gt.phase > 1 ):
            gt.phase -= 1 # Make sure we roll over properly, decrement the phase until it is < 1

        # Clear distance_foot_traveled list
        self.distance_foot_traveled = np.zeros( hrd.num_legs )

        # Pass that phase in to grab our desired foot positions
        foot_pos, foot_off_ground = self.foot_trajectory_planning( walk_twist.walk_direction )
        joint_ang = foot_position_to_joint_angles( foot_pos ) # Grab joint angles from foot_position

        # Package leg state info into message
        for leg in range( hrd.num_legs ):
            self.lg_st_msg[leg].foot_off_ground = foot_off_ground[leg]
            self.lg_st_msg[leg].foot_position.x = foot_pos[0,leg]
            self.lg_st_msg[leg].foot_position.y = foot_pos[1,leg]
            self.lg_st_msg[leg].foot_position.z = foot_pos[2,leg]
            self.lg_st_msg[leg].joint_angle.position = joint_ang[:,leg]
            self.pub[leg].publish( self.lg_st_msg[leg] ) # Publish the joint angles

        self.t_previous = t

    #==============================================================================
    def foot_trajectory_planning( self, twist ):

        # Initialize foot position array
        foot_position = np.zeros( shape=(3,6) )
        foot_off_ground = np.zeros( 6, dtype=bool )

        for leg in range( 6 ):

            # Find at what point in the period the foot contacts the ground again
            phase_end = gt.phase_offset[leg] + 1/3
            if phase_end > 1:
                phase_end = phase_end - 1 # Leg 4 leg up wraps around to begin of period

            # Determine if this leg is up or down
            if  (gt.phase >= gt.phase_offset[leg] and gt.phase < phase_end) or (leg == 4-1 and (gt.phase >= gt.phase_offset[leg] or gt.phase < phase_end)):
                # Leg up ------------------------------
                if leg == 4-1 and gt.phase < 1/6:
                    up_phase = gt.phase * 3 + 0.5
                else:
                    up_phase = (gt.phase - gt.phase_offset[leg]) / (1 - gt.beta)

                foot_off_ground[leg] = True
                foot_position[:,leg] = self.integrate_foot_up_pos( leg, twist, up_phase )

            else:
                # Leg down -----------------------------
                foot_off_ground[leg] = False
                foot_position[:,leg] = self.integrate_foot_ground_pos( leg, twist )

        return foot_position, foot_off_ground

    #==============================================================================
    def integrate_foot_ground_pos( self, leg, twist ):
        """ Uses commanded body twist vector, translate twist to desired foot current
        position, and uses calculated linear velocity and dt to integrate foot position
        Args:
            leg (int): Which leg to update foot position for. Expected range 0-5
            twist (geometry_msg/Twist): Commanded twist vector in body frame
        Returns:
            foot_pos_new (float(3)): Newly calculated foot position
        """

        # Current foot position
        p = np.array( [ self.lg_st_msg[leg].foot_position.x,
                        self.lg_st_msg[leg].foot_position.y,
                        self.lg_st_msg[leg].foot_position.z] )

        # Transformation from body twist to foot twist
        T_body2foot = np.eye( 6 )
        T_body2foot[3:,0:3] = rotation.skew( p )

        twist_body = np.array( [twist.angular.x, twist.angular.y, twist.angular.z,
                                -twist.linear.x, -twist.linear.y, -twist.linear.z] )
        twist_foot = T_body2foot @ twist_body

        # Track how far the foot is moving this time step
        self.distance_foot_traveled[leg] = np.linalg.norm( twist_foot[3:] ) * self.dt

        # Integrate foot position
        return p + twist_foot[3:] * self.dt

    #==============================================================================
    def integrate_foot_up_pos( self, leg, twist, up_phase ):

        if up_phase < self.last_lifting_phase[leg]:
            # This is the start of the up-phase, grab our starting foot position
            self.up_phase_foot_limit[:,leg] = np.array( [ self.lg_st_msg[leg].foot_position.x,
                                                          self.lg_st_msg[leg].foot_position.y,
                                                          self.lg_st_msg[leg].foot_position.z] )

        self.last_lifting_phase[leg] = up_phase

        new_foot_pos = [ np.interp( up_phase, [0,1.0], [self.up_phase_foot_limit[0,leg], gt.foot_center[0,leg]] ),
                         np.interp( up_phase, [0,1.0], [self.up_phase_foot_limit[1,leg], gt.foot_center[1,leg]] ),
                         -gt.foot_height * np.sin( np.pi * up_phase ) - gt.body_height ]

        return new_foot_pos


#==============================================================================
if __name__ == "__main__":
    try:
        gu = gait_update_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

