
class walk_topics:
    def __init__( self ):
        self.range_states = "range_states" # State info from the ultrasonic range sensor
        self.target_states = "target_states" # States from processed image info
        self.walk_twist = "walk_twist" # Commanded twist vector for walking gait
