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
        self.velocity = 0.020  # How fast the body moves, m/s (50mm/s ~= 2 in/s)
        self.stride_length = 0.080    # How far each stride should move, m
        self.stride_time = self.stride_length / ( self.velocity * self.beta ) # How long each strid lasts

        self.phase = 0

        self.phase_offset = np.array( [4.0/6.0, 1.0/6.0, 2.0/6.0, 5.0/6.0, 0.0, 3.0/6.0] )    # Which point in the period each foot lifts off

        self.foot_height = 0.04   # How high each foot will lift off the ground, m
        self.body_height = 0.05   # How high the body will be off the ground, m

        self.ground_x = np.array( [-0.15, 0.15, -0.2, 0.2, -0.15, 0.15] )   # How far away each leg is from the body center

        self.ground_y_max = np.array( [ 0.12 + self.stride_length, 0.12 + self.stride_length,
                                        self.stride_length/2, self.stride_length/2,
                                        -0.12, -0.12] )  # How far forward each foot will move, body frame, m

        self.ground_y_min = self.ground_y_max - self.stride_length

        self.walking = True