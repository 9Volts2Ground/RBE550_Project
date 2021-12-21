from cv2 import projectPoints
import numpy as np

# Plotting Libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

# Custom libraries
import hardware

hrd = hardware.hardware()

fig1 = plt.figure(1)
ax = fig1.add_subplot( 111, projection = '3d' )
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
fig1.suptitle('Foot Positions, Body Frame')
fig1.tight_layout()
fig1.show()

xrange = [-0.35, 0.35]
yrange = [-0.35, 0.35]
zrange = [-0.05, 0.05]

for leg in range(6):

    foot_position = hrd.s[:,leg]
    for q1 in np.linspace( hrd.motorMin[0,leg], hrd.motorMax[0,leg], 10 ):
        q1 = ( q1 - hrd.motorCenter[0,leg] ) * np.pi/180

        for q2 in np.linspace( hrd.motorMin[1,leg], hrd.motorMax[1,leg], 10 ):
            q2 = ( q2 - hrd.motorCenter[1,leg] ) * np.pi/180

            for q3 in np.linspace( hrd.motorMin[2,leg], hrd.motorMax[2,leg], 10 ):
                q3 = ( q3 - hrd.motorCenter[2,leg] ) * np.pi/180

                joint_angles = [q1, q2, q3]

                hrd.joint_angles[:,leg] = joint_angles
                T_b2f = hrd.fkine_body2foot( leg )

                foot_pos = T_b2f[0:3,3]
                if foot_pos[0] > xrange[0] and foot_pos[0] < xrange[1] and \
                    foot_pos[1] > yrange[0] and foot_pos[1] < yrange[1] and \
                    foot_pos[2] > zrange[0] and foot_pos[2] < zrange[1]:
                
                    foot_position = np.vstack( [ foot_position, foot_pos ] )

    ax.scatter( foot_position[:,0], foot_position[:,1], foot_position[:,2], label=str('Leg'+str(leg+1)), marker='.' )

ax.set_xlim( xrange )
ax.set_ylim( yrange )
ax.set_zlim( zrange )

ax.legend()
print("done...")
