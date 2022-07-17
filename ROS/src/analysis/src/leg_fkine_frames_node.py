#!/usr/bin/env python3
#=====================
from geometry_msgs.msg import TransformStamped
import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_from_matrix

# Custom libraries and class instances
from hardware_control.hw_topics import hw_topics
from hardware_control.msg import leg_states # Custom ROS message types
from hardware_control import hardware_constants
from walk_sense import transformation_frames

# Initialize classes
hrd = hardware_constants.hardware_constants()
hw_top = hw_topics()
tf = transformation_frames.transformation_frames()

#==============================================================================
class leg_fkine_frames_node():
    def __init__(self):
        rospy.init_node( "leg_fkine_frames_node", anonymous=True )

        self.tf2_broadcaster = [ tf2_ros.TransformBroadcaster() for leg in range( hrd.num_legs ) ]

        for leg in range( hrd.num_legs ):
            rospy.Subscriber( hw_top.leg_states[leg], leg_states, self.calc_leg_fkine )


    #==============================================================================
    def calc_leg_fkine( self, lg_st_msg ):

        T_body2shoulder = hrd.transform_body2shoulder( lg_st_msg.leg_num )

        # Calculate transforms to each joint
        T_shoulder2knee = hrd.transform_shoulder2knee( lg_st_msg.joint_angle.position[0] )
        T_knee2ankle    = hrd.transform_knee2ankle(    lg_st_msg.joint_angle.position[1] )
        T_ankle2foot    = hrd.transform_ankle2foot(    lg_st_msg.joint_angle.position[2] )

        # Calculate intermediate transforms
        T_body2knee = T_body2shoulder @ T_shoulder2knee
        # T_body2ankle = T_body2knee @ T_knee2ankle
        # T_body2foot = T_body2ankle @ T_ankle2foot

        # Broadcast body2knee
        self.broadcast_fkine( lg_st_msg.leg_num, T_body2knee, tf.body, tf.knee[ lg_st_msg.leg_num ] )

        # Broadcast knee2ankle
        self.broadcast_fkine( lg_st_msg.leg_num, T_knee2ankle, tf.knee[ lg_st_msg.leg_num ], tf.ankle[ lg_st_msg.leg_num ] )

        # Broadcast ankle2foot
        self.broadcast_fkine( lg_st_msg.leg_num, T_ankle2foot, tf.ankle[ lg_st_msg.leg_num ], tf.foot[ lg_st_msg.leg_num ] )

    #==============================================================================
    def broadcast_fkine( self, leg_num, T_x2y, parent, child ):
        trans = TransformStamped()
        trans.header.stamp = rospy.Time.now()

        # Package and send Tb2k
        trans.header.frame_id = parent
        trans.child_frame_id = child
        trans.transform.translation.x = T_x2y[0,3]
        trans.transform.translation.y = T_x2y[1,3]
        trans.transform.translation.z = T_x2y[2,3]

        q = quaternion_from_matrix( T_x2y )
        # q = quaternion_from_matrix( T_x2y[0:3, 0:3] )
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]

        self.tf2_broadcaster[leg_num].sendTransform( trans )

#==============================================================================
if __name__ == "__main__":
    try:
        fkine = leg_fkine_frames_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
