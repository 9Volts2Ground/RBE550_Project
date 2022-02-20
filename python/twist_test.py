
import numpy as np


from rotation import rotrx, rotry, rotrz, skew
from gait import gait

gt = gait()

R = rotrz( 0 )

twist_w = [0, 0, np.pi/6]
twist_v = [0, 1.0, 0]
twist_b = np.append( twist_w, twist_v )

for foot in range(6):
    p = gt.foot_center[:,foot]

    Tw_body_to_foot = np.zeros( (6,6) )
    Tw_body_to_foot[0:3,0:3] = R
    Tw_body_to_foot[3:,0:3] = skew( p ) @ R
    Tw_body_to_foot[3:,3:] = R

    twist_f = Tw_body_to_foot @ np.append( twist_b[0:3], -twist_b[3:] )

    print("foot = ", foot, "------------------")
    print("R = ", R)
    print("p = ", p)
    print("Tb = ", twist_b)
    print("Tf = ", twist_f)

print("done")


