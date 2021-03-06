#!/usr/bin/env python3
import numpy as np
import platform

# Custom libraries
from hardware_control import rotation

class hardware_constants:

    def __init__(self):

        # Flag to determine if running on robot hardware
        self.wanda = self.is_wanda()

        self.num_legs = 6
        self.num_joints = 3 # Number of joints in each leg
        self.num_seeker_joints = 2

        # Vector from body center to hip joints, NWU coordinate frames, meters
        self.s = np.array([ [ 0.076162, 0.055, 0.048 ],        # Front left
                            [ 0.076162, -0.055, 0.048 ],        # Front right
                            [ 0.0, 0.083376, 0.048 ],         # Middle left
                            [ 0.0, -0.083376, 0.048 ],         # Middle right
                            [ -0.076162, 0.055, 0.048 ],       # Back left
                            [ -0.076162, -0.055, 0.048 ] ] ).T  # Back right

        # Leg link lengths, m
        self.L1 = 0.03226     # Coxa
        self.L2 = 0.090      # Femur
        self.L3 = 0.113     # Tibia

        # Distance to link center of gravity.
        # Current assumption is it is along vector from joint to joint. Will update later
        self.servo_mass = 0.055 # kg
        self.L1_m = self.servo_mass * 2
        self.L1_cg = self.L1 / 2 # Assume cg is between the two servos. Neglect brackets for now

        self.L2_m = 0.01312952 # kg
        self.L2_cg = self.L2 / 2

        self.L3_link_mass = 0.04475333 # kg
        self.L3_m = self.L3_link_mass + self.servo_mass
        self.L3_cg = ( self.servo_mass * self.L3/4 + self.L3_link_mass * self.L3/3 ) / self.L3_m

        # PWM channel for each motor
        self.legChannel = np.array( [ [ 16, 17, 18 ],       # Front left
                                      [ 15, 14, 13 ],       # Front right
                                      [ 19, 20, 21 ],       # Middle left
                                      [ 12, 11, 10 ],       # Middle right
                                      [ 22, 23, 27 ],       # Back left
                                      [  9,  8, 31 ] ] ).T  # Back right

        # Motor angle centers, degrees
        self.motorCenter = np.array( [  [ 115, 110, 160 ],
                                        [ 120, 105, 20 ],
                                        [ 120, 110, 155 ],
                                        [ 120, 110, 26 ],
                                        [ 100, 110, 143 ],
                                        [ 125, 120, 25 ] ] ).T

        # Motor angle mins, degrees
        self.motorMin = np.array( [ [ 50, 20, 0 ],
                                    [ 85, 10, 0 ],
                                    [ 70, 15, 0 ],
                                    [ 70, 15, 0 ],
                                    [ 60, 20, 0 ],
                                    [ 60, 20, 0 ] ] ).T

        # Motor angle maxs, degrees
        self.motorMax = np.array( [ [ 150, 180, 180 ],
                                    [ 175, 180, 180 ],
                                    [ 160, 180, 180 ],
                                    [ 170, 180, 180 ],
                                    [ 160, 180, 180 ],
                                    [ 165, 180, 180 ] ] ).T

        # Motor angle command matches matches defined orientation angle
        self.motorOrientation = np.array( [ [ -1, -1, -1 ],
                                            [ -1,  1,  1 ],
                                            [ -1, -1, -1 ],
                                            [ -1,  1,  1 ],
                                            [ -1, -1, -1 ],
                                            [ -1,  1,  1 ] ] ).T

        self.alpha_offset = np.array( [ np.arctan2( self.s[1,0], self.s[0,0] ),
                                        np.arctan2( self.s[1,1], self.s[0,1] ),
                                        np.arctan2( self.s[1,2], self.s[0,2] ),
                                        np.arctan2( self.s[1,3], self.s[0,3] ),
                                        np.arctan2( self.s[1,4], self.s[0,4] ),
                                        np.arctan2( self.s[1,5], self.s[0,5] ) ] )

        #------------------------------------------------------------
        # Seeker info
        #------------------------------------------------------------
        # Transformation from body frame to el motor axis
        self.T_body2neck = np.array( [ [1.0, 0.0, 0.0, self.s[0,0]],
                                       [0.0, 1.0, 0.0, 0.0],
                                       [0.0, 0.0, 1.0, 0.13858],
                                       [0.0, 0.0, 0.0, 1.0] ] )

        # ToDo: measure new Scarlet Witch crown value from CAD model
        self.seeker_extension_length = 0.045 # Length of extension from el motor axis to camera face

        self.seeker_channel = np.array( [1, 0] )

        self.seekerCenter = np.array( [ 85, 91 ] )
        self.seekerMin = np.array( [ 20, 75 ] )
        self.seekerMax = np.array( [ 160, 180 ] )

        self.seekerOrientation = np.array( [1, 1] )

        #------------------------------------------------------------
        # LED info
        #------------------------------------------------------------
        self.leg2led = [6, 0, 5, 1, 4, 2] # Mapping between leg and LED index
        self.seeker_led = 3 # LED array number
    #==========================================================================
    def is_wanda( self ):
        uname = platform.uname()
        if "aarch64" in uname.processor:
            return True
        else:
            return False

    #==========================================================================
    def transform_body2shoulder( self, leg ):
        """
        Args:
            leg (integer): Which leg to get transformation for
        Returns:
            T_body2shoulder [4x4 np array]: Transformation from body frame to shoulder joint of leg
        """

        c_alpha = np.cos( self.alpha_offset[leg] )
        s_alpha = np.sin( self.alpha_offset[leg] )

        T_body2shoulder = np.array( [[ c_alpha, -s_alpha, 0.0, self.s[0,leg] ],
                                     [ s_alpha,  c_alpha, 0.0, self.s[1,leg] ],
                                     [     0.0,      0.0, 1.0, self.s[2,leg] ],
                                     [     0.0,      0.0, 0.0,           1.0 ] ] )
        return T_body2shoulder

    #==========================================================================
    def transform_shoulder2knee( self, angle, link_length = None ):
        """Gets transformation matrix from shoulder joint to knee joint.
        Option to change link length, like to use link CG instead of full length.
        Defaults to L1
        Args:
            angle (float): Joint angle of shoulder joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L1.
        Returns:
            4x4 numpy array: Transformation matrix
        """
        if link_length is None:
            link_length = self.L1

        c_shoulder = np.cos( angle )
        s_shoulder = np.sin( angle )

        return np.array( [ [ c_shoulder, 0.0,  s_shoulder, link_length*c_shoulder ],
                           [ s_shoulder, 0.0, -c_shoulder, link_length*s_shoulder ],
                           [ 0.0,        1.0,         0.0,                     0.0],
                           [ 0.0,        0.0,         0.0,                    1.0 ] ] )

    #==========================================================================
    def transform_knee2ankle( self, angle, link_length = None ):
        """Gets transformation matrix from knee joint to ankle joint.
        Option to change link length, like to use link CG instead of full length.
        Defaults to L2
        Args:
            angle (float): Joint angle of knee joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L2.
        Returns:
            4x4 numpy array: Transformation matrix
        """

        if link_length is None:
            link_length = self.L2

        c_knee = np.cos( angle )
        s_knee = np.sin( angle )

        return np.array( [ [ c_knee,  s_knee,  0.0, link_length*c_knee ],
                           [ s_knee, -c_knee,  0.0, link_length*s_knee ],
                           [ 0.0,        0.0, -1.0,                0.0 ],
                           [ 0.0,        0.0,  0.0,                1.0 ] ] )

    #==========================================================================
    def transform_ankle2foot( self, angle, link_length = None ):
        """Gets transformation matrix from ankle joint to foot.
        Option to change link length, like to use link CG instead of full length.
        Defaults to L3
        Args:
            angle (float): Joint angle of ankle joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L3.
        Returns:
            4x4 numpy array: Transformation matrix
        """

        if link_length is None:
            link_length = self.L3

        c_ankle = np.cos( angle )
        s_ankle = np.sin( angle )

        return np.array( [ [ c_ankle, -s_ankle, 0.0, link_length*c_ankle ],
                           [ s_ankle,  c_ankle, 0.0, link_length*s_ankle ],
                           [ 0.0,          0.0, 1.0,                 0.0 ],
                           [ 0.0,          0.0, 0.0,                 1.0 ] ] )

    #==========================================================================
    def rotation_inertial2body( self, body_state_inertial ):
        """Uses individual Euler angles, calculates rotation matrix between inertial and body frames.
        From body frame: rotate yaw, pitch roll
        """

        R_roll = rotation.rotry( body_state_inertial[11] )
        R_pitch = rotation.rotrx( body_state_inertial[10] )
        R_yaw = rotation.rotrz( body_state_inertial[9])

        return R_roll @ R_pitch @ R_yaw # Body orientation matrix in inertial frame

    #==========================================================================
    def transform_inertial2body( self, body_state_inertial ):
        """
        Calculates transformation matrix from inertial to body frame
        Returns: T_inertial2body [4x4 numpy array]
        """
        T_inertial2body = np.identity(4)
        T_inertial2body[0:3,0:3] = self.rotation_inertial2body( body_state_inertial )
        T_inertial2body[0:3,3] = body_state_inertial[0:3]

        return T_inertial2body

#==============================================================================
if __name__ == "__main__":
    hrd = hardware_constants()
    print("done")
