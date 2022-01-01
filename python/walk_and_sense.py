import multiprocessing
from multiprocessing import Process, Lock, Value, Array, Event
import numpy as np
import os
import time

# Custom libraries
import gait
import hardware
from hardware_control_process import hardware_control_process
from walk_forward import walk_forward

# Multiprocessing example:
# https://pretagteam.com/question/sharing-object-class-instance-using-multiprocessing-managers
# Helpful documentation:
# https://docs.python.org/3/library/multiprocessing.html

def walk_and_sense( plot = False, log = True ):


    # Initialize hardware
    hrd = hardware.hardware()

    # Need some static hardware parameters to pass into walking algorithm
    alpha_offset = hrd.alpha_offset
    s_vecs = hrd.s
    leg_links = [hrd.L1, hrd.L2, hrd.L3]

    # Set the body height
    gt = gait.gait()
    hrd.body_state_inertial[2] = gt.body_height

    #--------------------------------------------------------------------------
    # Define shared memory between processes
    # velocity = Value('d', gt.velocity)
    body_state_inertial = Array('d', hrd.body_state_inertial.flatten() )
    foot_position = Array('d', hrd.foot_position.flatten() )
    joint_angles  = Array('d', hrd.joint_angles.flatten() )
    foot_off_ground = Array('d', hrd.foot_off_ground.flatten() )
    walk_forward_updated = Value('b', False)
    walk_status = Value('b', True)
    started_walking = Event()
    seeker_angles = Array('d', hrd.seeker_angles.flatten())
    measured_range = Value('d', 0.0)
    T_b2c = Array('d', hrd.T_body2cam.flatten())
    T_i2b = Array('d', np.identity(4).flatten())
    # Seeker events
    move_seeker_request = Event()
    seeker_done_moving = Event()
    take_range_measure = Event()
    range_measure_returned = Event()
    # Kalman Filter events
    heading = Array('d', [0.0, gt.velocity, 0.0])
    request_IMU_data = Event()
    IMU_data_ready = Event()
    accel_measured = Array('d', hrd.IMU.accel_measured)
    gyro_measured = Array('d', hrd.IMU.gyro_measured)
    #--------------------------------------------------------------------------

    # Keeps output orderly
    lock = Lock()

    # Define walking start/stop times
    t_start = time.time()
    t_end = t_start + 3

    #--------------------------------------------------------------------------
    # Define processes
    hrd_control = Process( target=hardware_control_process, args=(
        hrd,
        gt,
        t_start,
        gt.velocity, # Multiprocess value, v * dt
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
        plot,
        log,) )

    walk = Process( target=walk_forward, args=(
        gt,
        t_start,
        t_end,
        alpha_offset, # Angle of each shoulder joint relative to body frame
        s_vecs, # Vector to each shoulder from body frame
        leg_links, # Link lengths of legs
        walk_status, # Multiprocess flag, to set if still walking
        started_walking, # Multiprocessing Event
        walk_forward_updated, # Multiprocess flag, did walk_forward() run an update?
        foot_position, # Multiprocess array
        foot_off_ground, # Multiprocess array, flag for hardware LED
        joint_angles,) ) # Multiprocess array

    #--------------------------------------------------------------------------
    # Run processes
    hrd_control.start()
    walk.start()

    hrd_control.join()
    walk.join()


    print("ending...")

if __name__ == "__main__":
    pwd = os.getcwd()

    if "wanda" in pwd:
        plot = False
        log = True
    else:
        plot = True
        log = True

    # Call gait design function to calculate joint angles
    walk_and_sense( plot, log )


