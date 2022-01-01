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
        self.velocity = 0.030  # How fast the body moves, m/s (50mm/s ~= 2 in/s)
        self.stride_length = 0.080    # How far each stride should move, m
        self.stride_time = self.stride_length / ( self.velocity * self.beta ) # How long each strid lasts

        self.phase = 0

        self.phase_offset = np.array( [4.0/6.0, 1.0/6.0, 2.0/6.0, 5.0/6.0, 0.0, 3.0/6.0] )    # Which point in the period each foot lifts off

        # Redesigned for strafe
        self.body_height = 0.03   # How high the body will be off the ground, m
        self.foot_center = np.array( [ [-0.16, 0.21, -self.body_height],
                                       [0.16, 0.21, -self.body_height],
                                       [-0.27, 0.0, -self.body_height],
                                       [0.27, 0.0, -self.body_height],
                                       [-0.16, -0.21, -self.body_height],
                                       [0.16, -0.21, -self.body_height] ] ).T
        self.foot_height = -0.01 # Z position of foot at peak stride in body frame, meters

        self.stride_radius = 0.035 # meters

        #----------------------------------------------
        # Variables used for different types of gaites
        self.gait_type = "curved"
        # self.gait_type = "straight"
        #------------------------
        # Variables for "straight" gaites
        self.strafe_angle = 0.0 # Direction of strafe movement, angle around Z body axis, radians
        #------------------------
        # Variables for "curved" gaites
        self.curve_center = np.array( [0,0] )  # x,y coordinates in body frame of point around which to curve around
        self.Rn = np.zeros(6) # Radius from gait curve center point to each foot center
        self.intersect_point = np.zeros((2,2,6)) # Coordinates of intersection between foot circle and curve radius circle
        self.intersect_angle = np.zeros((2,6)) # Angle from body X axis to intersection points of foot area and curve radius, radians

        self.walking = True

        self.init_curved_values()

    #==============================================================================
    def set_curve_radius( self, x, y ):
        self.curve_center = np.array( [x, y] )
        self.gait_type = "curved"
        self.init_curved_values()

    #==============================================================================
    def init_curved_values( self ):

        for leg in range(6):
            self.Rn[leg] = np.linalg.norm( self.curve_center[0:2] - self.foot_center[0:2,leg] )

            # Find intersection points
            Rn2 = self.Rn[leg]**2
            for cord, add in enumerate( [1, -1] ):
                self.intersect_point[0,cord,leg] = (self.curve_center[0] + self.foot_center[0,leg])/2 + \
                                                (self.foot_center[0,leg] - self.curve_center[0]) * \
                                                (Rn2 - self.stride_radius**2)/(2 * Rn2) + \
                                                add * 0.5 * (self.curve_center[1] - self.foot_center[1,leg]) / Rn2 * \
                                                np.sqrt(4 * Rn2 * self.stride_radius**2 - self.stride_radius**4)

                self.intersect_point[1,cord,leg] = (self.curve_center[1] + self.foot_center[1,leg])/2 + \
                                                (self.foot_center[1,leg] - self.curve_center[1]) * \
                                                (Rn2 - self.stride_radius**2)/(2 * Rn2) - \
                                                add * 0.5 * (self.curve_center[0] - self.foot_center[0,leg]) / Rn2 * \
                                                np.sqrt(4 * Rn2 * self.stride_radius**2 - self.stride_radius**4)

                # Find angles to start and end
                self.intersect_angle[cord,leg] = np.arctan2( self.intersect_point[1,cord,leg] - self.curve_center[1],
                                                             self.intersect_point[0,cord,leg] - self.curve_center[0] )

            # if self.velocity > 0:
            #     if self.curve_center[0] > self.foot_center[0,leg] and self.intersect_angle[0,leg] > self.intersect_angle[1,leg]:
            #         self.switch_point_order( leg ) # 1
            #     elif self.curve_center[0] < self.foot_center[0,leg] and self.intersect_angle[0,leg] < self.intersect_angle[1,leg]:
            #         self.switch_point_order( leg ) # 2
            #     elif (self.curve_center[0] == self.foot_center[0,leg] or self.curve_center[1] == self.foot_center[1,leg]) and self.intersect_angle[0,leg] < self.intersect_angle[1,leg]:
            #         self.switch_point_order( leg ) # 5
            # elif self.velocity < 0:
            #     if self.curve_center[0] > self.foot_center[0,leg] and self.intersect_angle[0,leg] < self.intersect_angle[1,leg]:
            #         self.switch_point_order( leg ) # 3
            #     elif self.curve_center[0] < self.foot_center[0,leg] and self.intersect_angle[0,leg] > self.intersect_angle[1,leg]:
            #         self.switch_point_order( leg ) # 4
            #     elif (self.curve_center[0] == self.foot_center[0,leg] or self.curve_center[1] == self.foot_center[1,leg]) and self.intersect_angle[0,leg] > self.intersect_angle[1,leg]:
            #         self.switch_point_order( leg ) # 6



            if self.velocity > 0:
                if self.curve_center[0] > self.foot_center[0,leg] and self.curve_center[1] == self.foot_center[1,leg]:
                    if self.intersect_angle[0,leg] > self.intersect_angle[1,leg]:
                        print("Option 1")
                        self.switch_point_order( leg )
                elif (self.curve_center[0] == self.foot_center[0,leg] or self.curve_center[1] == self.foot_center[1,leg]) and self.intersect_angle[0,leg] < self.intersect_angle[1,leg]:
                    print("Option 2")
                    self.switch_point_order( leg )
                elif self.intersect_angle[0,leg] < self.intersect_angle[1,leg]:
                    print("Option 3")
                    self.switch_point_order( leg )
            elif self.velocity < 0:
                if self.curve_center[0] > self.foot_center[0,leg] and self.curve_center[1] == self.foot_center[1,leg] and self.intersect_angle[0,leg] < self.intersect_angle[1,leg]:
                    self.switch_point_order( leg )
                elif (self.curve_center[0] == self.foot_center[0,leg] or self.curve_center[1] == self.foot_center[1,leg]) and self.intersect_angle[0,leg] > self.intersect_angle[1,leg]:
                    self.switch_point_order( leg )
                elif self.intersect_angle[0,leg] > self.intersect_angle[1,leg]:
                    self.switch_point_order( leg )


    #--------------------------------------------------------------------------
    def switch_point_order( self, leg ):

        print("Fixing leg:", leg)

        # Switch the order
        self.intersect_angle[:,leg] = [self.intersect_angle[1,leg], self.intersect_angle[0,leg]]

        temp_intersect = np.zeros((2,2))
        temp_intersect[:,:] = self.intersect_point[:,:,leg]

        self.intersect_point[0,0,leg] = temp_intersect[0,1]
        self.intersect_point[1,0,leg] = temp_intersect[1,1]
        self.intersect_point[0,1,leg] = temp_intersect[0,0]
        self.intersect_point[1,1,leg] = temp_intersect[1,0]
