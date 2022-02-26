import numpy as np

class gait:

    # Make this class a singleton
    __instance = None
    def __new__(cls, *args, **kwargs):
        if not gait.__instance:
            gait.__instance = object.__new__(cls)
        return gait.__instance

    def __init__(self):
        self.beta = 4.0/6.0 # Ratio of how long each foot is on the ground

        self.phase = 0 # Initialize to 0, but increment as it walks

        # Which point in the period each foot lifts off
        self.phase_offset = np.array( [4.0/6.0, 1.0/6.0, 2.0/6.0, 5.0/6.0, 0.0, 3.0/6.0] )

        # Redesigned for strafe
        self.body_height = 0.05   # How high the body will be relative to the ground, m
        self.foot_center = np.array( [ [-0.14, 0.17, -self.body_height],
                                       [0.14, 0.17, -self.body_height],
                                       [-0.20, 0.0, -self.body_height],
                                       [0.20, 0.0, -self.body_height],
                                       [-0.14, -0.17, -self.body_height],
                                       [0.14, -0.17, -self.body_height] ] ).T
        self.foot_height = -0.01 # Z position of foot at peak stride in body frame, meters

        self.max_stride_length = 0.068 # meters
