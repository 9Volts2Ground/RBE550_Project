import numpy as np

class hardware_constants:

    def __init__(self):
        
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


