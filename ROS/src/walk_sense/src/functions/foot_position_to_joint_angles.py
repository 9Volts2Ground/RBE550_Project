#!/usr/bin/env python3
import numpy as np
from tf.transformations import quaternion_matrix

# Custom libraries
from hardware_control import hardware_constants
from hardware_control import rotation

hrd = hardware_constants.hardware_constants()

#==============================================================================
def foot_position_to_joint_angles( foot_position_g, body_ground_transform ):
    """Calculates joint angles for all legs, given the position
    of all feet in the body_ground frame, and the transform
    between the body_ground and body frames.
    Args:
        foot_position_g[3,6] (real): Position of each foot [cord,leg]
            in the body_ground frame
    Returns:
        joint_angles[3,6] (real): Joint angles for each leg to
            achieve the desired foot position [joint,leg]
    """

    # Grab transformation matrix between body_ground and body frames
    Tbg = quaternion_matrix( [ body_ground_transform.transform.rotation.x,
                               body_ground_transform.transform.rotation.y,
                               body_ground_transform.transform.rotation.z,
                               body_ground_transform.transform.rotation.w ] )
    Tbg[0:3,3] = -np.array( [ body_ground_transform.transform.translation.x,
                              body_ground_transform.transform.translation.y,
                              body_ground_transform.transform.translation.z ] ) # Mind your negatives



    # Grab joint angles from foot_position_g
    joint_angles = np.zeros( shape=(3,6) ) # Initialize array
    for leg in range(6):

        Rz = rotation.rotrz( hrd.alpha_offset[leg] )

        # Grab transformation from Wanda body frame to shoulder[leg] frame
        Tsb = np.identity( 4 )
        Tsb[0:3,0:3] = Rz.T
        Tsb[0:3,3] = -Rz.T @ hrd.s[:,leg]

        foot_vec_ground = np.append( foot_position_g[:,leg], 1.0 )

        position_shoulder_frame = Tsb @ Tbg @ foot_vec_ground.T

        joint_angles[:,leg] = ikine_leg( position_shoulder_frame[0:3] )

    return joint_angles

#==============================================================================
def ikine_leg( pos ):
    """Given array of [x,y,z] foot coordinates in the leg shoulder frame,
    calculate shoulder, knee, and ankle joint angles necessary to achieve
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

    joint_angles[1] = -( gamma - beta ) # Negative causes knee-up configuration

    return joint_angles
