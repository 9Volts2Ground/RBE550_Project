#!/usr/bin/env python3

num_legs = 6

class hw_topics:
    def __init__(self):

        # Leg topics
        self.leg_states = []
        for leg in range( num_legs ):
            self.leg_states.append( f"leg_states{leg}" )

        # Seeker-related topics
        self.seeker_states = 'seeker_states'
        self.camera_image = 'camera_image'

