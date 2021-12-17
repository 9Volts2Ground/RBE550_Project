import numpy as np

# Custom libraries
import hardware
from ikine_leg import ikine_leg
from rotation import rotrz

def foot_position_to_joint_angles(foot_position, alpha_offset, s_vecs, leg_links):
    # Grab joint angles from foot_position
    joint_angles = np.zeros( shape=(3,6) )
    for leg in range(6):

        Rz = rotrz( alpha_offset[leg])

        # Grab transformation from Wanda body frame to shoulder[leg] frame
        T = np.identity( 4 )
        T[0:3,0:3] = Rz.T
        T[0:3,3] = -Rz.T @ s_vecs[:,leg]

        s_array = np.append(foot_position[:,leg], 1.0 )

        position_shoulder_frame = T @ s_array.T

        angles = ikine_leg( leg_links, position_shoulder_frame[0:3] )
        joint_angles[:,leg] = angles

    return joint_angles

