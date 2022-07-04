
from hardware_control import hardware_constants

hrd = hardware_constants.hardware_constants()

class transformation_frames:
    def __init__( self ):
        self.ground = "ground" # Ground frame, placed under robot body. Walking gait calculates in this frame
        self.body = "body" # Robot body frame, defined at base of robot hardware. Referenced from ground frame
        
        # Each foot gets its own transform frame
        self.foot = [f"foot{leg}" for leg in range( hrd.num_legs )]
