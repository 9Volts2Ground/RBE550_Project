from multiprocessing import Process, Value, Array
import numpy as np
import pandas as pd
import os
import sys
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

from plot_seeker_sweep_map import plot_seeker_sweep_map

def seeker_sweep_map(
    walk_status, # Multiprocessing flag
    seeker_angles, # Multiprocess array
    measured_range, # Multiprocess value
    T_b2c, # Multiprocessing array, transformation between body and camera
    T_i2b, # Multiprocess array, transformation between inertial and body
    move_seeker_request, # Multiprocess event, seeker needs to do stuff
    seeker_done_moving, # Multiprocess event, seeker has been told to move
    take_range_measure, # Multiprocess event, seeker needs to take measurement
    range_measure_returned, # Multiprocess event, seeker has obtained a range
    plot = False,
    log = True
    ):

    # Define seeker sweep parameters
    az_steps = 12
    sweep_right_range = np.linspace( -60*np.pi/180, 60*np.pi/180, az_steps )
    sweep_left_range = -sweep_right_range
    sweep_dir = np.array( [-1,1] )

    el = 0.0 # For now, just keep seeker elevation at 0 degrees

    # Plotting variables
    points_body_frame = np.array( [0.0, 0.0, 0.0] )
    skr_body_frame = np.array( [0.0, 0.0, 0.0] )
    points_inertial_frame = np.array( [0.0, 0.0, 0.0] )
    skr_inertial_frame = np.array( [0.0, 0.0, 0.0] )

    while walk_status.value:

        for az in np.linspace( sweep_dir[0]*60*np.pi/180, sweep_dir[1]*60*np.pi/180, az_steps ):

            # Make sure we quit the seeker sweep if we finish walking
            if not walk_status.value:
                break

            seeker_angles[:] = [az, el]

            # Tell the seeker to move
            move_seeker_request.set()

            # Wait for the seeker to finish moving
            seeker_done_moving.wait()
            seeker_done_moving.clear()

            # Tell the seeker to take a range measure
            take_range_measure.set()

            # Wait for a range measurement to return
            range_measure_returned.wait()
            range_measure_returned.clear()


            range = measured_range.value
            print("Range: ", range, "+++++++++++++++")

            T_body2cam = np.reshape( T_b2c, (4,4) )
            point_body_frame = T_body2cam @ np.array( [range, 0.0, 0.0, 1.0] ).T
            seeker_body_frame = T_body2cam[:,3]

            T_inertial2body = np.reshape( T_i2b, (4,4) )
            point_inertial_frame = T_inertial2body @ point_body_frame.T
            seeker_inertial_frame = T_inertial2body @ seeker_body_frame.T

            # ToDo: Run some analysis to decide if the body is about to impact an identified point

            if plot or log:
                points_body_frame = np.vstack( ( points_body_frame, point_body_frame[0:3] ) )
                skr_body_frame = np.vstack( (skr_body_frame, seeker_body_frame[0:3] ) )
                points_inertial_frame = np.vstack( (points_inertial_frame, point_inertial_frame[0:3]) )
                skr_inertial_frame = np.vstack( (skr_inertial_frame, seeker_inertial_frame[0:3]))

        sweep_dir = sweep_dir * -1 # Sweep the seeker back the other direction now

    #==========================================================================
    # Plot the data at the end

    if plot or log:
        seeker_data = pd.DataFrame({
            'P_BOD_X':points_body_frame[:,0],
            'P_BOD_Y':points_body_frame[:,1],
            'P_BOD_Z':points_body_frame[:,2],
            'SKR_BOD_X':skr_body_frame[:,0],
            'SKR_BOD_Y':skr_body_frame[:,1],
            'SKR_BOD_Z':skr_body_frame[:,2],
            'P_INERT_X':points_inertial_frame[:,0],
            'P_INERT_Y':points_inertial_frame[:,1],
            'P_INERT_Z':points_inertial_frame[:,2],
            'SKR_INERT_X':skr_inertial_frame[:,0],
            'SKR_INERT_Y':skr_inertial_frame[:,1],
            'SKR_INERT_Z':skr_inertial_frame[:,2],
        })

        seeker_data.to_csv( os.path.join( "logged_data", "seeker_data.csv" ) )

        if plot:
            plot_seeker_sweep_map( seeker_data )

    print("seeker done")
