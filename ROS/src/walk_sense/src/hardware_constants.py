import numpy as np
import platform

# Custom libraries
from rotation import rotrx, rotry, rotrz

class hardware_constants:

    def __init__(self):

        # Flag to determine if running on robot hardware
        self.wanda = self.is_wanda()

        # Vector from body center to hip joints, meters
        self.s = np.array([ [ -0.055, 0.076162, 0.048 ],        # Front left
                            [  0.055, 0.076162, 0.048 ],        # Front right
                            [ -0.083376, 0.0, 0.048 ],         # Middle left
                            [  0.083376, 0.0, 0.048 ],         # Middle right
                            [ -0.055, -0.076162, 0.048 ],       # Back left
                            [  0.055, -0.076162, 0.048 ] ] ).T  # Back right

        # Leg link lengths, m
        self.L1 = 0.03226     # Coxa
        self.L2 = 0.090      # Femur
        self.L3 = 0.113     # Tibia

        # PWM channel for each motor
        self.legChannel = np.array( [ [ 16, 17, 18 ],       # Front left
                                      [ 15, 14, 13 ],       # Front right
                                      [ 19, 20, 21 ],       # Middle left
                                      [ 12, 11, 10 ],       # Middle right
                                      [ 22, 23, 27 ],       # Back left
                                      [  9,  8, 31 ] ] ).T  # Back right

        # Motor angle centers, degrees
        self.motorCenter = np.array( [  [ 115, 100, 160 ],
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
        self.T_body2neck = np.array( [ [0.0, -1.0, 0.0, 0.0],
                                       [1.0,  0.0, 0.0, self.s[1,0]],
                                       [0.0,  0.0, 1.0, 0.13858],
                                       [0.0,  0.0, 0.0, 1.0] ] )

        self.seeker_extension_length = 0.03226 # Length of extension from el motor axis to camera face

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
        if "ARM" in uname.processor:
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

        T_body2shoulder = np.array( [[ np.cos(self.alpha_offset[leg]), -np.sin(self.alpha_offset[leg]), 0, self.s[0,leg] ],
                                     [ np.sin(self.alpha_offset[leg]),  np.cos(self.alpha_offset[leg]), 0, self.s[1,leg] ],
                                     [ 0.0, 0.0, 1.0, self.s[2,leg] ],
                                     [ 0.0, 0.0, 0.0, 1.0 ] ] )
        return T_body2shoulder

    #==========================================================================
    def rotation_inertial2body( self, body_state_inertial ):
        """Uses individual Euler angles, calculates rotation matrix between inertial and body frames.
        From body frame: rotate yaw, pitch roll
        """

        R_roll = rotry( body_state_inertial[11] )
        R_pitch = rotrx( body_state_inertial[10] )
        R_yaw = rotrz( body_state_inertial[9])

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



