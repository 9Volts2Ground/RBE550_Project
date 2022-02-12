#!/usr/bin/env python3

num_legs = 6

class channels:
    def __init__(self):
        
        self.inertial_state = "inertial_state"

        self.leg_states = []
        for leg in range( num_legs ):
            self.leg_states.append( f"leg_states{leg}" )

