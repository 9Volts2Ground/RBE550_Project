import argparse
import numpy as np

#==============================================================================
def rotrx( theta ):
    """ Calculates 3x3 rotation matrix around X axis
    Args:
        theta (float): Angle of rotation in radians
    Returns:
        R (3x3 numpy array): Rotation matrix
    """
    cost = np.cos(theta)
    sint = np.sin(theta)
    R = np.array( [ [1.0, 0.0, 0.0],
                    [0.0, cost, -sint],
                    [0.0, sint,  cost] ] )

    return R

#==============================================================================
def rotry( theta ):
    """ Calculates 3x3 rotation matrix around Y axis

    Args:
        theta (float): Angle of rotation in radians

    Returns:
        R (3x3 numpy array): Rotation matrix
    """
    cost = np.cos(theta)
    sint = np.sin(theta)
    R = np.array( [ [cost, 0.0, sint],
                    [0.0, 1.0, 0.0],
                    [-sint, 0.0, cost] ] )

    return R

#==============================================================================
def rotrz( theta ):
    """ Calculates 3x3 rotation matrix around Z axis

    Args:
        theta (float): Angle of rotation in radians

    Returns:
        R (3x3 numpy array): Rotation matrix
    """
    cost = np.cos(theta)
    sint = np.sin(theta)
    R = np.array( [ [cost, -sint, 0.0],
                    [sint,  cost, 0.0],
                    [0.0, 0.0, 1.0] ] )

    return R

#==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument( '-t', '--theta', help='Angle of rotation about Z, radians', default=0.0)

    args = parser.parse_args()

    R = rotrz( float( args.theta ) )

    print("R = " + str(R))
