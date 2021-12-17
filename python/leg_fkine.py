import numpy as np 

def leg_fkine( joint_angles, leg_links ):
    
    shoulder = joint_angles[0]
    knee = joint_angles[1]
    ankle = joint_angles[2]
    
    L1 = leg_links[0]
    L2 = leg_links[1]
    L3 = leg_links[2]
    
    T_shoulder2foot = np.array( [ [ np.cos(shoulder)*np.cos(ankle - knee), -np.cos(shoulder)*np.sin(ankle - knee), -np.sin(shoulder), np.cos(shoulder)*(L1 + L2*np.cos(knee) + L3*np.cos(ankle - knee))],
                                  [ np.sin(shoulder)*np.cos(ankle - knee), -np.sin(shoulder)*np.sin(ankle - knee),  np.cos(shoulder), np.sin(shoulder)*(L1 + L2*np.cos(knee) + L3*np.cos(ankle - knee))],
                                  [              -np.sin(ankle - knee),               -np.cos(ankle - knee),              0,                      L2*np.sin(knee) - L3*np.sin(ankle - knee)],
                                  [                               0,                                0,              0,                                                        1] ] )
    
    
    

