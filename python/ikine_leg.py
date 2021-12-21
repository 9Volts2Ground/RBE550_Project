import argparse
import numpy as np
# Custom libraries
import hardware

def ikine_leg( leg_links, pos ):

    joint_angles = np.array( [0.0, 0.0, 0.0] )

    joint_angles[0] = np.arctan2( pos[1], pos[0] )

    # Length of base of triangle from q2 to end effector
    r = np.linalg.norm( np.array( pos[0:2] ) ) - leg_links[0]

    D = ( r**2 + pos[2]**2 - leg_links[1]**2 - leg_links[2]**2 ) / ( 2 * leg_links[1] * leg_links[2] )

    joint_angles[2] = np.arctan2( np.sqrt( 1 - D**2 ), D )

    gamma = np.arctan2( -pos[2], r )
    beta = np.arctan2( leg_links[2] * np.sin( joint_angles[2] ), \
        leg_links[1] + leg_links[2]*np.cos( joint_angles[2] ) )

    joint_angles[1] = -( gamma - beta )

    return joint_angles

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument( '-p', '--pos', help='Desired osition vector, length 3', default=[0.0, 0.0, 0.0] )

    args = parser.parse_args()

    pos = np.array( args.pos )

    joint_angles = ikine_leg( pos )

    print( "Joint angles: " + str( joint_angles) )