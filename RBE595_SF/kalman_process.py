from multiprocessing import Value, Event
import numpy as np
import os
import pandas as pd
import time

from Kalman import Kalman
from plot_kalman_data import plot_kalman_data

def kalman_process(
    t_start,
    walk_status, # Multiprocess value
    started_walking,
    body_state_inertial,
    grav_vector, # Multiprocess array
    gyro_bias, # Multiprocess array
    accel_std, # Multiprocess value # Maybe just pass these in initially?
    gyro_std, # Multiprocess value
    heading, # Multiprocess array
    accel_measured, # Multiprocess array
    gyro_measured, # Multiprocess array
    request_IMU_data, # Multiprocess event
    IMU_data_ready, # Multiprocess event
    plot = False,
    log = True
    ):

    # Plotting vectors
    euler = np.zeros(3)
    angular_velocity = np.zeros(3)
    position = np.zeros(3)
    velocity = np.zeros(3)
    time_vec = [0]
    z = np.zeros(6)
    accel = np.zeros(3)
    gyro = np.zeros(3)
    grav = np.zeros(3)
    gyro_offset = np.zeros(3)

    started_walking.wait()

    kalm = Kalman( dim_x = 15,
                    dim_z = 6,
                    g = grav_vector[:],
                    gyro_bias = gyro_bias[:] )

    kalm.x = np.array( body_state_inertial[:] )

    kalm.update_R( np.max(accel_std)**2, np.max(gyro_std)**2 )

    update_rate = 1/50

    t = time.time()
    previous_time = t
    while walk_status.value:


        if t - previous_time >= update_rate:

            dt = t - previous_time
            previous_time = t

            # Request IMU data from the hardware process
            request_IMU_data.set()

            # Wait until the IMU data is ready
            IMU_data_ready.wait()
            IMU_data_ready.clear()

            # Kalman Filter processing
            kalm.prediction_update( heading[:], dt )
            kalm.correction_update( accel_measured[:], gyro_measured[:] )

            # Save values to plot
            if plot or log:
                # Store states for plotting
                time_vec.append( t - t_start )
                position = np.vstack( [position, kalm.x[0:3] ] )
                velocity = np.vstack( [velocity, kalm.x[3:6] ] )
                euler = np.vstack( [euler, kalm.x[9:12] ] )
                angular_velocity = np.vstack( [angular_velocity, kalm.x[12:15] ] )
                accel = np.vstack( [accel, accel_measured[:] ])
                gyro = np.vstack( [gyro, gyro_measured[:] ])
                z = np.vstack( [z, kalm.z])
                grav = np.vstack([grav, grav_vector])
                gyro_offset = np.vstack([gyro_offset, gyro_bias])



        t = time.time()

    if plot or log:

        kalman_data = pd.DataFrame( {
            'TIME':time_vec,
            'P_X':position[:,0],
            'P_Y':position[:,1],
            'P_Z':position[:,2],
            'V_X':velocity[:,0],
            'V_Y':velocity[:,1],
            'V_Z':velocity[:,2],
            'YAW':euler[:,0],
            'PITCH':euler[:,1],
            'ROLL':euler[:,2],
            'YAW_D':angular_velocity[:,0],
            'PITCH_D':angular_velocity[:,1],
            'ROLL_D':angular_velocity[:,2],
            'A_XM':accel[:,0],
            'A_YM':accel[:,1],
            'A_ZM':accel[:,2],
            'Y_DM':gyro[:,0],
            'P_DM':gyro[:,1],
            'R_DM':gyro[:,2],
            'Z_AX':z[:,0],
            'Z_AY':z[:,1],
            'Z_AZ':z[:,2],
            'Z_GX':z[:,3],
            'Z_GY':z[:,4],
            'Z_GZ':z[:,5],
            'G_X':grav[:,0],
            'G_Y':grav[:,1],
            'G_Z':grav[:,2],
            'GYRO_BX':gyro_offset[:,0],
            'GYRO_BY':gyro_offset[:,1],
            'GYRO_BZ':gyro_offset[:,2],
            } )

        kalman_data.to_csv( os.path.join( "logged_data", "kalman_data.csv" ) )

        if plot:
            plot_kalman_data( kalman_data )


    print("Done Kalman")
