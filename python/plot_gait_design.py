import numpy as np

# Plotting Libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

# Custom libraries
import gait

gt = gait.gait()

# Initialize the figure
fig1 = plt.figure(1)
ax = fig1.add_subplot()
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.axis("equal")
fig1.suptitle("Gait Design")
fig1.tight_layout()
fig1.show()

theta = np.linspace( 0, 2*np.pi, 100 )
circle_x = gt.stride_radius * np.cos(theta)
circle_y = gt.stride_radius * np.sin(theta)

# Loop through all legs, plot gait design
for leg in range(6):
    ax.plot( gt.foot_center[0,leg], gt.foot_center[1,leg], marker="*", color="k")
    ax.plot( gt.foot_center[0,leg] + circle_x, gt.foot_center[1,leg] + circle_y, color='b')

    ax.plot( [gt.curve_center[0], gt.intersect_point[0,0,leg]], [gt.curve_center[1], gt.intersect_point[1,0,leg]], color='r' )
    ax.plot( [gt.curve_center[0], gt.intersect_point[0,1,leg]], [gt.curve_center[1], gt.intersect_point[1,1,leg]], color='g' )
    
    ax.plot( gt.intersect_point[0,0,leg], gt.intersect_point[1,0,leg], color='r', marker="*" )
    ax.plot( gt.intersect_point[0,1,leg], gt.intersect_point[1,1,leg], color='g', marker="*" )


print("done")
