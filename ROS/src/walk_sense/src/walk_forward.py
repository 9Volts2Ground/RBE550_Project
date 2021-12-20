#!/usr/bin/env python3
#=====================
import numpy as np
import rospy
import time

# Custom libraries
import channels # List of acceptable channel names
from foot_position_to_joint_angles import foot_position_to_joint_angles
import gait
from walk_sense.msg import leg_states # Custom ROS message types

chn = channels.channels()
gt = gait.gait()

#==============================================================================
def walk_forward():

    # Initialize ROS communication
    pub = rospy.Publisher( chn.leg_states, leg_states, queue_size = 1 )
    rospy.init_node( "walk_forward", anonymous=True )
    rate = rospy.Rate( 50 ) # 50 Hz
    lg_st_msg = leg_states()

    # Initialize walking variables
    update_rate = 1/100
    t_start = time.time()
    t = t_start
    previous_time = t_start

    print("Entering walking while loop...")

    while not rospy.is_shutdown():

        # Check to 100 Hz update
        if t - previous_time >= update_rate:

            # Integrate inertial pose as pose = pose + v * dt
            # Assume just forward velocity in Y direction, body frame
            # fwd_displacement.value = gt.velocity * (t - previous_time)
            # print("fwd_displacement: ", fwd_displacement.value)
            previous_time = t

            # Find what part of the gait phase we are in
            gt.phase = ( (t-t_start) % gt.stride_time ) / gt.stride_time

            # Pass that phase in to grab our desired foot positions
            foot_pos, foot_off_ground = foot_trajectory_planning()
            lg_st_msg.foot_off_ground = foot_off_ground.tolist()
            lg_st_msg.foot_position = foot_pos.flatten().tolist() # convert 3x6 into 3*6

            # Grab joint angles from foot_position
            joint_ang = foot_position_to_joint_angles(foot_pos)
            lg_st_msg.joint_angles = joint_ang.flatten().tolist() # Convert 3x6 to 3*6
            # Publish the joint angles
            pub.publish( lg_st_msg )

        t = time.time()

#==============================================================================
def foot_trajectory_planning():

    # Initialize foot position array
    foot_position = np.zeros( shape=(3,6) )
    foot_off_ground = np.zeros( 6, dtype=int )

    for leg in range( 6 ):

        # Find at what point in the period the foot contacts the ground again
        phase_end = gt.phase_offset[leg] + 1/3
        if phase_end > 1:
            # Leg 4 leg up wraps around to begin of period
            phase_end = phase_end - 1

        # Determine if this leg is up or down
        if  (gt.phase >= gt.phase_offset[leg] and gt.phase < phase_end) or (leg == 4-1 and (gt.phase >= gt.phase_offset[leg] or gt.phase < phase_end)):
            # Leg up ------------------------------

            if leg == 4-1 and gt.phase < 1/6:
                up_phase = gt.phase * 3 + 0.5
            else:
                up_phase = (gt.phase - gt.phase_offset[leg]) / (1 - gt.beta)

            foot_position[:,leg] = [gt.ground_x[leg],
                                    ( gt.ground_y_max[leg] - gt.ground_y_min[leg] ) * up_phase + gt.ground_y_min[leg],
                                    gt.foot_height * np.sin( np.pi * up_phase ) - gt.body_height ]

            foot_off_ground[leg] = 1


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

            foot_position[:,leg] = [gt.ground_x[leg],
                                    gt.ground_y_max[leg] - (gt.ground_y_max[leg] - gt.ground_y_min[leg] ) * down_phase,
                                    -gt.body_height]

            foot_off_ground[leg] = 0

    return foot_position, foot_off_ground

#==============================================================================
if __name__ == "__main__":
    try:
        walk_forward()
    except rospy.ROSInterruptException:
        pass

