import numpy as np
import time
from multiprocessing import Value

# Custom libraries
import hardware
from foot_position_to_joint_angles import foot_position_to_joint_angles
from foot_trajectory_planning import foot_trajectory_planning
import gait
from rotation import rotrz

def walk_forward( gt,
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
                  joint_angles): # Multiprocess array

    walk_status.value = True # Multiprocess flag

    # Scheduler variables
    update_rate = 1/100
    t_length = 5
    t_final = t_start + t_length

    t = t_start
    previous_time = t_start
    gait_start = time.time()
    while t <= t_end:

        if time.time() - gait_start >= 0.5 and not started_walking.is_set():
            started_walking.set()

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
            foot_pos, foot_off_ground[:] = foot_trajectory_planning( gt.phase, gt )
            foot_position[:] = foot_pos.flatten() # convert 3x6 into 3*6

            # Grab joint angles from foot_position
            joint_ang = foot_position_to_joint_angles(foot_pos, alpha_offset, s_vecs, leg_links)
            joint_angles[:] = joint_ang.flatten() # Convert 3x6 to 3*6

            # Send flag so hardware processes and updates its states
            walk_forward_updated.value = True

        # Update current time
        t = time.time()

    gt.walking = False

    walk_status.value = False

    print("walking done")
