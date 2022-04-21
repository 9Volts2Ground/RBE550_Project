
class walk_topics:
    def __init__( self ):
        self.range_states = "range_states" # State info from the ultrasonic range sensor
        self.target_states = "target_states" # States from processed image info
        self.target_track = "target_track" # Target track state info
        self.walk_twist = "walk_twist" # Commanded twist vector for walking gait
        self.processed_image = "processed_image" # Image after processing target search states
        self.range_sensor = "range_sensor" # Measurements from ultrasonic range sensor
