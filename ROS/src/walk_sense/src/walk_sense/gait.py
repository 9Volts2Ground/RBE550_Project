import numpy as np

#====================================================================
class gait:
    #=======================================
    # Make this class a singleton
    __instance = None
    def __new__(cls, *args, **kwargs):
        if not gait.__instance:
            gait.__instance = object.__new__(cls)
        return gait.__instance

    #=======================================
    def __init__(self):
        self.beta = 4.0/6.0 # Ratio of how long each foot is on the ground

        self.phase = 0 # Initialize to 0, but increment as it walks

        # Which phase in the stride period each foot lifts off the ground
        self.phase_offset = np.array( [4.0/6.0, 1.0/6.0, 2.0/6.0, 5.0/6.0, 0.0, 3.0/6.0] )

        # Redesigned for strafe
        self.body_height = 0.05   # How high the body will be relative to the ground, m

        # Where the foot will center around, relative to the ground frame
        self.foot_center = np.array( [ [0.17,   0.14, 0.0],
                                       [0.17,  -0.14, 0.0],
                                       [0.0,    0.20, 0.0],
                                       [0.0,   -0.20, 0.0],
                                       [-0.17,  0.14, 0.0],
                                       [-0.17, -0.14, 0.0] ] ).T
        self.foot_height = 0.04 # Z position of foot at peak stride in ground frame, meters

        self.max_stride_length = 0.068 # meters
