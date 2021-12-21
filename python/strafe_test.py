import numpy as np 

radius = 0.035 # Radius of stride circle

x_center = 0.0
y_center = 0.0

theta = 0.0 # Strafe angle
theta = -np.pi/2

for phase in np.linspace( 0, 1, 20):
    r_phase = (2*phase - 1) * radius
    
    # Phase when down
    pos = [np.sin(theta) * r_phase + x_center, 
           -np.cos(theta) * r_phase + y_center]
    
    # print(pos)
    
    # Phase when up
    pos = [-np.sin(theta) * r_phase + x_center, 
           np.cos(theta) * r_phase + y_center]
    print(pos)
    
    
    
