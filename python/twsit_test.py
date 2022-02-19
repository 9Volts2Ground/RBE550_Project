
import numpy as np


from rotation import rotrx, rotry, rotrz, skew
from gait import gait

gt = gait()

p = gt.foot_center[:,0]

R = rotrz( 0 )

twist_w = [0, 0, np.pi/6]
twist_v = [0, 0.1, 0]
twist_b = np.append( twist_w, twist_v )

Tw_body_to_foot = np.zeros( (6,6) )
Tw_body_to_foot[0:3,0:3] = R
Tw_body_to_foot[3:,0:3] = skew( p ) @ R
Tw_body_to_foot[3:,3:] = R

twist_f = Tw_body_to_foot @ twist_b

print("done")


