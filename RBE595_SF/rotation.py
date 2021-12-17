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

    R = np.array( [ [1.0, 0.0, 0.0],
                    [0.0, np.cos(theta), -np.sin(theta)],
                    [0.0, np.sin(theta),  np.cos(theta)] ] )

    return R

#==============================================================================
def rotry( theta ):
    """ Calculates 3x3 rotation matrix around Y axis

    Args:
        theta (float): Angle of rotation in radians

    Returns:
        R (3x3 numpy array): Rotation matrix
    """

    R = np.array( [ [np.cos(theta), 0.0, np.sin(theta)],
                    [0.0, 1.0, 0.0],
                    [-np.sin(theta), 0.0, np.cos(theta)] ] )

    return R

#==============================================================================
def rotrz( theta ):
    """ Calculates 3x3 rotation matrix around Z axis

    Args:
        theta (float): Angle of rotation in radians

    Returns:
        R (3x3 numpy array): Rotation matrix
    """

    R = np.array( [ [np.cos(theta), -np.sin(theta), 0.0],
                    [np.sin(theta),  np.cos(theta), 0.0],
                    [0.0, 0.0, 1.0] ] )

    return R

#==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument( '-t', '--theta', help='Angle of rotation about Z, radians', default=0.0)

    args = parser.parse_args()

    R = rotrz( float( args.theta ) )

    print("R = " + str(R))
