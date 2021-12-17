import numpy as np
from numpy.core.shape_base import _block_check_depths_match
# Custom classes
import gait
import hardware

def foot_trajectory_planning( phase, gait ):

    # Initialize foot position array
    foot_position = np.zeros( shape=(3,6) )
    foot_off_ground = np.zeros( 6 )

    for leg in range( 6 ):

        # Find at what point in the period the foot contacts the ground again
        phase_end = gait.phase_offset[leg] + 1/3
        if phase_end > 1:
            # Leg 4 leg up wraps around to begin of period
            phase_end = phase_end - 1

        # Determine if this leg is up or down
        if  (phase >= gait.phase_offset[leg] and phase < phase_end) or (leg == 4-1 and (phase >= gait.phase_offset[leg] or phase < phase_end)):
            # Leg up ------------------------------

            if leg == 4-1 and phase < 1/6:
                up_phase = phase * 3 + 0.5
            else:
                up_phase = (phase - gait.phase_offset[leg]) / (1 - gait.beta)

            foot_position[:,leg] = [gait.ground_x[leg],
                                    ( gait.ground_y_max[leg] - gait.ground_y_min[leg] ) * up_phase + gait.ground_y_min[leg],
                                    gait.foot_height * np.sin( np.pi * up_phase ) - gait.body_height ]

            foot_off_ground[leg] = 1


        else:
            # Leg down -----------------------------

            if leg == 4-1:
                down_phase = (phase - 1/6) / gait.beta
            elif phase >= gait.phase_offset[leg] + 1.0/3.0:
                # After foot touch down
                down_phase = (phase - (gait.phase_offset[leg] + 1/3 ) ) / gait.beta
            else:
                # Before foot lift off
                down_phase = (phase + 1 - (gait.phase_offset[leg] + 1/3)) / gait.beta

            foot_position[:,leg] = [gait.ground_x[leg],
                                    gait.ground_y_max[leg] - (gait.ground_y_max[leg] - gait.ground_y_min[leg] ) * down_phase,
                                    -gait.body_height]

            foot_off_ground[leg] = 0

    return foot_position, foot_off_ground



