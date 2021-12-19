#!/usr/bin/env python3

import rospy
from walk_sense.msg import joint_angles


def walk_forward():
    
    pub = rospy.Publisher( "joint_angles", joint_angles, que_size = 1 )
    
    rospy.init_node( walk_forward, anonymous=True )
    
    rate = rospy.Rate( 50 ) # 50 Hz
    
    while not rospy.is_shutdown():
        
        # Do walking stuff
        

if __name__ == "__main__":
    try:
        walk_forward()
    except rospy.ROSInterruptException:
        pass

