#!/usr/bin/env python3
#=====================
import copy
from geometry_msgs.msg import TransformStamped
import numpy as np
import rospy

import matplotlib.pyplot as plt
plt.style.use( 'seaborn-whitegrid' )

# Custom libraries and class instances
from classes.walk_topics import walk_topics
from hardware_control.hw_topics import hw_topics
from hardware_control.msg import leg_states # Custom ROS message types
from hardware_control import hardware_constants

# Initialize classes
hrd = hardware_constants.hardware_constants()
hw_top = hw_topics()
w_top = walk_topics()

#==============================================================================
class stability_analysis_node():
    def __init__(self):
        rospy.init_node( "stability_analysis_node", anonymous=True )

        # State info from other publishers
        self.body_pose = TransformStamped()
        self.lg_st_msg = []
        for leg in range( hrd.num_legs ):
            self.lg_st_msg.append( leg_states() )

        self.leg_link_cgs = np.zeros( ( 3, 3, 6 ) ) # xyz, link, leg
        self.foot_pos = np.zeros( ( 3, 6 ) )

        for leg in range( hrd.num_legs ):
            rospy.Subscriber( hw_top.leg_states[leg], leg_states, self.grab_leg_states )

        self.stability_analysis( )

    #==========================================================================
    def stability_analysis( self ):
        # Set loop rate, jump into the loop
        rate = rospy.Rate( 20 )

        full_cg = np.zeros(3)

        while not rospy.is_shutdown():
            # Grab states for each foot
            foot_pos = copy.deepcopy( self.foot_pos )
            leg_link_cgs = copy.deepcopy( self.leg_link_cgs )

            # Calculate center of gravity
            for itr in range( 3 ):
                full_cg[itr] = np.mean(leg_link_cgs[itr,:,:])

            print(f"full_cg = {full_cg}")

    #==========================================================================
    def grab_leg_states( self, lg_st_msg ):
        self.lg_st_msg[lg_st_msg.leg_num] = copy.deepcopy( lg_st_msg )

        T_body2shoulder = hrd.transform_body2shoulder( lg_st_msg.leg_num )

        c_shoulder = np.cos( lg_st_msg.joint_angle.position[0] )
        s_shoulder = np.sin( lg_st_msg.joint_angle.position[0] )
        c_knee = np.cos( lg_st_msg.joint_angle.position[1] )
        s_knee = np.sin( lg_st_msg.joint_angle.position[1] )
        c_ankle = np.cos( lg_st_msg.joint_angle.position[2] )
        s_ankle = np.sin( lg_st_msg.joint_angle.position[2] )

        # Create transformations to CGs
        T_shoulder2L1_cg = np.array( [ [ c_shoulder, 0.0, s_shoulder, hrd.L1_cg*c_shoulder ],
                                      [ s_shoulder, 0.0, -c_shoulder, hrd.L1_cg*s_shoulder ],
                                      [ 0.0, 1.0, 0.0, 0.0],
                                      [ 0.0, 0.0, 0.0, 1.0 ] ] )
        T_body2L1_cg = T_body2shoulder @ T_shoulder2L1_cg

        T_shoulder2knee = np.array( [ [ c_shoulder, 0.0, s_shoulder, hrd.L1*c_shoulder ],
                                      [ s_shoulder, 0.0, -c_shoulder, hrd.L1*s_shoulder ],
                                      [ 0.0, 1.0, 0.0, 0.0],
                                      [ 0.0, 0.0, 0.0, 1.0 ] ] )

        T_knee2L2_cg = np.array( [ [ c_knee, s_knee, 0.0, hrd.L2_cg*c_knee ],
                                   [ s_knee, -c_knee, 0.0, hrd.L2_cg*s_knee ],
                                   [ 0.0, 0.0, -1.0, 0.0 ],
                                   [ 0.0, 0.0, 0.0, 1.0 ]] )
        T_body2knee = T_body2shoulder @ T_shoulder2knee
        T_body2L2_cg = T_body2knee @ T_knee2L2_cg

        T_knee2ankle = np.array( [ [ c_knee, s_knee, 0.0, hrd.L2*c_knee ],
                                   [ s_knee, -c_knee, 0.0, hrd.L2*s_knee ],
                                   [ 0.0, 0.0, -1.0, 0.0 ],
                                   [ 0.0, 0.0, 0.0, 1.0 ]] )
        T_ankle2L3_cg = np.array( [ [ c_ankle, -s_ankle, 0.0, hrd.L3_cg*c_ankle ],
                                    [ s_ankle, c_ankle, 0.0, hrd.L3_cg*s_ankle],
                                    [ 0.0, 0.0, 1.0, 0.0 ],
                                    [ 0.0, 0.0, 0.0, 1.0 ] ] )
        T_body2ankle = T_body2knee @ T_knee2ankle
        T_body2L3_cg = T_body2ankle @ T_ankle2L3_cg

        T_ankle2foot = np.array( [ [ c_ankle, -s_ankle, 0.0, hrd.L3*c_ankle ],
                                   [ s_ankle, c_ankle, 0.0, hrd.L3*s_ankle],
                                   [ 0.0, 0.0, 1.0, 0.0 ],
                                   [ 0.0, 0.0, 0.0, 1.0 ] ] )
        T_body2foot = T_body2ankle @ T_ankle2foot

        # Extract link CG's from transformations
        self.leg_link_cgs[:,0,lg_st_msg.leg_num] = T_body2L1_cg[0:3,3]
        self.leg_link_cgs[:,1,lg_st_msg.leg_num] = T_body2L2_cg[0:3,3]
        self.leg_link_cgs[:,2,lg_st_msg.leg_num] = T_body2L3_cg[0:3,3]

        # Do fkine to get foot positions in body frame
        self.foot_pos[:,lg_st_msg.leg_num] = T_body2foot[0:3,3]


#==============================================================================
if __name__ == "__main__":
    try:
        stab_anal = stability_analysis_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
