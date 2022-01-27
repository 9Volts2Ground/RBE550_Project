#!/usr/bin/env python3
import numpy as np

# Custom libraries
from classes import hardware_constants
from functions.rotation import rotrz

hrd = hardware_constants.hardware_constants()

#==============================================================================
def foot_position_to_joint_angles( foot_position ):
    # Grab joint angles from foot_position
    joint_angles = np.zeros( shape=(3,6) )
    for leg in range(6):

        Rz = rotrz( hrd.alpha_offset[leg])

        # Grab transformation from Wanda body frame to shoulder[leg] frame
        T = np.identity( 4 )
        T[0:3,0:3] = Rz.T
        T[0:3,3] = -Rz.T @ hrd.s[:,leg]

        s_array = np.append( foot_position[:,leg], 1.0 )

        position_shoulder_frame = T @ s_array.T

        joint_angles[:,leg] = ikine_leg( position_shoulder_frame[0:3] )

    return joint_angles

#==============================================================================
def ikine_leg( pos ):
    """Given array of [x,y,z] foot coordinates in the leg shoulder frame,
    calculate shoulder, knee, ankle joint angles necessary to achieve
    the desired pose

    Args:
        pos[3] (float): array of x,y,z foot position in shoulder frame, meters

    Returns:
        joint_angles[3] (float): shoulder, knee, ankle joint angles, radians
    """

    joint_angles = np.array( [0.0, 0.0, 0.0] ) # Initialize array

    joint_angles[0] = np.arctan2( pos[1], pos[0] )

    # Length of base of triangle from q2 to end effector
    r = np.linalg.norm( np.array( pos[0:2] ) ) - hrd.L1

    D = ( r**2 + pos[2]**2 - hrd.L2**2 - hrd.L3**2 ) / ( 2 * hrd.L2 * hrd.L3 )

    joint_angles[2] = np.arctan2( np.sqrt( 1 - D**2 ), D )

    gamma = np.arctan2( -pos[2], r )
    beta = np.arctan2( hrd.L3 * np.sin( joint_angles[2] ), \
        hrd.L2 + hrd.L3*np.cos( joint_angles[2] ) )

    joint_angles[1] = -( gamma - beta )

    return joint_angles
