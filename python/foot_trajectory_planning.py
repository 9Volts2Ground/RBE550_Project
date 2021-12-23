import numpy as np
# Custom classes
import gait

def foot_trajectory_planning( phase, gait ):
    """Given our phase in the gait, returns desired foot position

    Args:
        phase (float): Phase of stide, from 0:1
        gait (class): Class containing gait design parameters
    Returns:
        foot_position (3x6 float): x,y,z coordinates for each foot
        foot_off_ground (6): array to flag if foot is on or off the ground
    """

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

            foot_off_ground[leg] = 1

            # Calculate position of foot at this phase
            foot_position[:,leg] = move_straight( up_phase, leg, foot_off_ground[leg], gait )

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

            foot_off_ground[leg] = 0

            # Calculate position of foot on the ground at this phase
            foot_position[:,leg] = move_straight( down_phase, leg, foot_off_ground[leg], gait )

    return foot_position, foot_off_ground


#==============================================================================
def move_straight( sub_phase, leg, foot_off_ground, gait ):

    if foot_off_ground:
        # Calculate position of foot at this phase
        r_phase = (2 * sub_phase - 1) * gait.stride_radius
        foot_position = [-np.sin(gait.strafe_angle) * r_phase + gait.foot_center[0,leg],
                        np.cos(gait.strafe_angle) * r_phase + gait.foot_center[1,leg],
                        (gait.foot_height - gait.foot_center[2,leg]) * np.sin(np.pi*sub_phase) + gait.foot_center[2,leg] ]

    else:
        # Calculate position of foot on the ground at this phase
        r_phase = (2 * sub_phase - 1) * gait.stride_radius
        foot_position = [ np.sin(gait.strafe_angle) * r_phase + gait.foot_center[0,leg],
                                -np.cos(gait.strafe_angle) * r_phase + gait.foot_center[1,leg],
                                    gait.foot_center[2,leg] ]
    return foot_position



