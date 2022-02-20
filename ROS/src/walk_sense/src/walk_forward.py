#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

# Custom libraries and class instances
from hardware_control.topics import topics # List of acceptable channel names
from hardware_control import hardware_constants
from classes import gait
from functions.foot_position_to_joint_angles import *
from hardware_control.msg import leg_states # Custom ROS message types

# Initialize classes
top = topics()
gt = gait.gait()
hrd = hardware_constants.hardware_constants()

#==============================================================================
def walk_forward():

    # Initialize ROS communication
    rospy.init_node( "walk_forward", anonymous=True )
    rate = rospy.Rate( 50 ) # Hz

    # Publish a state of each leg to its own topic
    lg_st_msg = []
    pub = []
    for leg in range( hrd.num_legs ):
        pub.append( rospy.Publisher( top.leg_states[leg], leg_states, queue_size = 1 ) )
        lg_st_msg.append( leg_states() )
        lg_st_msg[leg].leg_num = leg
        lg_st_msg[leg].joint_angle.position = [0.0, 0.0, 0.0]

    print("Entering walking while loop...")

    # Initialize walking variables
    t_start = rospy.get_time()

    while not rospy.is_shutdown():
        t = rospy.get_time()

        # Find what part of the gait phase we are in
        gt.phase = ( (t-t_start) % gt.stride_time ) / gt.stride_time

        # Pass that phase in to grab our desired foot positions
        foot_pos, foot_off_ground = foot_trajectory_planning()
        joint_ang = foot_position_to_joint_angles( foot_pos ) # Grab joint angles from foot_position

        # Package leg state info into message
        for leg in range( hrd.num_legs ):
            lg_st_msg[leg].foot_off_ground = foot_off_ground[leg]
            lg_st_msg[leg].foot_position.x = foot_pos[0,leg]
            lg_st_msg[leg].foot_position.y = foot_pos[1,leg]
            lg_st_msg[leg].foot_position.z = foot_pos[2,leg]
            lg_st_msg[leg].joint_angle.position = joint_ang[:,leg]
            pub[leg].publish( lg_st_msg[leg] ) # Publish the joint angles

        rate.sleep()

#==============================================================================
def foot_trajectory_planning():

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
            foot_position[:,leg] = [gt.ground_x[leg],
                                    ( gt.ground_y_max[leg] - gt.ground_y_min[leg] ) * up_phase + gt.ground_y_min[leg],
                                    gt.foot_height * np.sin( np.pi * up_phase ) - gt.body_height ]

        else:
            # Leg down -----------------------------
            if leg == 4-1:
                down_phase = (gt.phase - 1/6) / gt.beta
            elif gt.phase >= gt.phase_offset[leg] + 1.0/3.0:
                # After foot touch down
                down_phase = (gt.phase - (gt.phase_offset[leg] + 1/3 ) ) / gt.beta
            else:
                # Before foot lift off
                down_phase = (gt.phase + 1 - (gt.phase_offset[leg] + 1/3)) / gt.beta

            foot_off_ground[leg] = False
            foot_position[:,leg] = [gt.ground_x[leg],
                                    gt.ground_y_max[leg] - (gt.ground_y_max[leg] - gt.ground_y_min[leg] ) * down_phase,
                                    -gt.body_height]

    return foot_position, foot_off_ground

#==============================================================================
if __name__ == "__main__":
    try:
        walk_forward()
    except rospy.ROSInterruptException:
        pass

