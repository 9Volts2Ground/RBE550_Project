from multiprocessing import Value, Array
import numpy as np
import os
import pandas as pd
import time

# Custom libraries
import hardware
from plot_foot_trajectory import plot_foot_trajectory


def hardware_control_process(
        hrd,
        t_start,
        velocity, # Multiprocess value, v * dt
        body_state_inertial, # Multiprocess array
        foot_position, # Multiprocess array
        joint_angles, # Multiprocess array
        foot_off_ground, # Multiprocess array, flag for hardware LED
        walk_forward_updated, # Multiprocess flag, does walk_forward() have more data for us?
        walk_status, # Multiprocess flag, are we still moving?
        seeker_angles, # Multiprocessing array
        measured_range, # Multiprocessing value
        T_b2c, # Multiprocessing array
        T_i2b, # Multiprocess array
        move_seeker_request, # Multiprocess event, seeker needs to do stuff
        seeker_done_moving, # Multiprocess event, seeker has been told to move
        take_range_measure, # Multiprocess event, seeker needs to take measurement
        range_measure_returned, # Multiprocess event, seeker has obtained a range
        request_IMU_data, # Multiprocess event, for Kalman filter
        accel_measured, # Multiprocessing array
        gyro_measured, # Multiprocessing array
        IMU_data_ready, # Multiprocessing event
        plot = False,
        log = True
        ):

    # Set up plotting flags
    first_angle = True
    t_plot = np.array( [] )
    foot_pos = []
    joint_ang = []
    foot_position_inertial = []
    pose = []

    # Other initialization
    t = t_start
    t_previous = t_start
    seeker_actuated_time = t_start

    # Keep checking hardare updates until we stop walking
    while walk_status.value:

        #----------------------------------------------------
        # Has the walking algorithm given us any updates?
        if walk_forward_updated.value:

            t = time.time()
            print( "hrd_control.walk: ", t-t_start )
            walk_forward_updated.value = False # Clear the flag so we can re-update next time

            dt = t - t_previous
            fwd_displacement = velocity * dt
            t_previous = t

            inertial_displacement = hrd.rotation_inertial2body() @ np.array([0, fwd_displacement, 0]).T

            hrd.body_state_inertial[0:3] += inertial_displacement[0:3]

            body_state_inertial[0:3] = hrd.body_state_inertial[0:3] # Update the global value of this

            # print("foot_position: ", foot_position[:])
            hrd.foot_position = np.reshape( foot_position[:], (3,6) ) # Multiprocess flag, Body frame
            hrd.foot_off_ground = np.array( foot_off_ground[:] ) # Multiprocess flag, Flags
            hrd.update_foot_position_inertial()

            # Update joint angles, command motor actuation if running on hardware
            hrd.joint_angles = np.reshape( joint_angles[:], (3,6) )
            hrd.move_all_joints() # Command motor actuation

            # Store off data to plot later if desired
            if plot or log:
                t_plot = np.append( t_plot, time.time()-t_start)
                if first_angle:
                    foot_pos = hrd.foot_position
                    joint_ang = hrd.joint_angles
                    foot_position_inertial = hrd.foot_position_inertial
                    pose = hrd.body_state_inertial[0:3]
                    first_angle = False
                else:
                    foot_pos = np.dstack( [foot_pos, hrd.foot_position] )
                    joint_ang = np.dstack( [joint_ang, hrd.joint_angles] )
                    foot_position_inertial = np.dstack( [foot_position_inertial, hrd.foot_position_inertial])
                    pose = np.vstack( [pose, hrd.body_state_inertial[0:3] ])

        #----------------------------------------------------
        # Handle IMU data collection
        if request_IMU_data.is_set():
            hrd.IMU.get_current_measurement()

            accel_measured[:] = hrd.IMU.accel_averaged
            gyro_measured[:] = hrd.IMU.gyro_averaged

            request_IMU_data.clear()
            IMU_data_ready.set()



        #----------------------------------------------------
        # Has the seeker been told to actuate?
        if move_seeker_request.is_set():

            print( "hrd_control.skr: ", time.time()-t_start )

            hrd.seeker_angles[:] = seeker_angles[:] # Grab seeker angles from seeker process
            hrd.move_seeker() # Actuate the motors
            T_b2c[:] = hrd.T_body2cam.flatten()
            T_i2b[:] = hrd.transform_inertial2body().flatten()

            move_seeker_request.clear()
            seeker_done_moving.set()

        # Do we need to take a range measurement now?
        if time.time() >= hrd.seeker_actuated_time + 0.15 and take_range_measure.is_set():

            print( "hrd_control.range: ", time.time()-t_start )

            measured_range.value, range_stddev = hrd.average_range(3)

            take_range_measure.clear()
            range_measure_returned.set()


    # Make sure to set your seeker flags so it can finish up when we are done
    seeker_done_moving.set()
    range_measure_returned.set()
    IMU_data_ready.set()

    # When we're all done, see if we need to make plots
    if plot or log:

        walking_data = pd.DataFrame({
            'TIME':t_plot,
            'FOOT_1_X':foot_pos[0,0,:],
            'FOOT_1_Y':foot_pos[1,0,:],
            'FOOT_1_Z':foot_pos[2,0,:],
            'FOOT_2_X':foot_pos[0,1,:],
            'FOOT_2_Y':foot_pos[1,1,:],
            'FOOT_2_Z':foot_pos[2,1,:],
            'FOOT_3_X':foot_pos[0,2,:],
            'FOOT_3_Y':foot_pos[1,2,:],
            'FOOT_3_Z':foot_pos[2,2,:],
            'FOOT_4_X':foot_pos[0,3,:],
            'FOOT_4_Y':foot_pos[1,3,:],
            'FOOT_4_Z':foot_pos[2,3,:],
            'FOOT_5_X':foot_pos[0,4,:],
            'FOOT_5_Y':foot_pos[1,4,:],
            'FOOT_5_Z':foot_pos[2,4,:],
            'FOOT_6_X':foot_pos[0,5,:],
            'FOOT_6_Y':foot_pos[1,5,:],
            'FOOT_6_Z':foot_pos[2,5,:],
            'JOINT_1_X':joint_ang[0,0,:],
            'JOINT_1_Y':joint_ang[1,0,:],
            'JOINT_1_Z':joint_ang[2,0,:],
            'JOINT_2_X':joint_ang[0,1,:],
            'JOINT_2_Y':joint_ang[1,1,:],
            'JOINT_2_Z':joint_ang[2,1,:],
            'JOINT_3_X':joint_ang[0,2,:],
            'JOINT_3_Y':joint_ang[1,2,:],
            'JOINT_3_Z':joint_ang[2,2,:],
            'JOINT_4_X':joint_ang[0,3,:],
            'JOINT_4_Y':joint_ang[1,3,:],
            'JOINT_4_Z':joint_ang[2,3,:],
            'JOINT_5_X':joint_ang[0,4,:],
            'JOINT_5_Y':joint_ang[1,4,:],
            'JOINT_5_Z':joint_ang[2,4,:],
            'JOINT_6_X':joint_ang[0,5,:],
            'JOINT_6_Y':joint_ang[1,5,:],
            'JOINT_6_Z':joint_ang[2,5,:],
            'FOOT_I_1_X':foot_position_inertial[0,0,:],
            'FOOT_I_1_Y':foot_position_inertial[1,0,:],
            'FOOT_I_1_Z':foot_position_inertial[2,0,:],
            'FOOT_I_2_X':foot_position_inertial[0,1,:],
            'FOOT_I_2_Y':foot_position_inertial[1,1,:],
            'FOOT_I_2_Z':foot_position_inertial[2,1,:],
            'FOOT_I_3_X':foot_position_inertial[0,2,:],
            'FOOT_I_3_Y':foot_position_inertial[1,2,:],
            'FOOT_I_3_Z':foot_position_inertial[2,2,:],
            'FOOT_I_4_X':foot_position_inertial[0,3,:],
            'FOOT_I_4_Y':foot_position_inertial[1,3,:],
            'FOOT_I_4_Z':foot_position_inertial[2,3,:],
            'FOOT_I_5_X':foot_position_inertial[0,4,:],
            'FOOT_I_5_Y':foot_position_inertial[1,4,:],
            'FOOT_I_5_Z':foot_position_inertial[2,4,:],
            'FOOT_I_6_X':foot_position_inertial[0,5,:],
            'FOOT_I_6_Y':foot_position_inertial[1,5,:],
            'FOOT_I_6_Z':foot_position_inertial[2,5,:],
            'BODY_X':pose[:,0],
            'BODY_Y':pose[:,1],
            'BODY_Z':pose[:,2],
        })

        output_dir = os.path.join( os.path.dirname(__file__), "logged_data" )
        if not os.path.exists(output_dir):
            os.makedirs( output_dir )
        walking_data.to_csv( os.path.join( output_dir, "foot_trajectory_data.csv"))

        if plot:
            plot_foot_trajectory(
                walking_data,
                hrd.motorMin,
                hrd.motorMax,
                hrd.motorCenter,
                hrd.motorOrientation
                )

    print("hardware done")


