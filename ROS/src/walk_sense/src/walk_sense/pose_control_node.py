#!/usr/bin/env python3
#=====================
import numpy as np
import rospy
import time
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Custom libraries
from gait import gait
from transformation_frames import transformation_frames
from walk_topics import walk_topics

gt = gait()
tf = transformation_frames()
w_top = walk_topics()

#==============================================================================
class pose_control_node():
    def __init__(self):
        rospy.init_node( "pose_control_node", anonymous=True )

        self.pose = TransformStamped()
        self.pose.header.frame_id = tf.ground
        self.pose.child_frame_id = tf.body

        # tf2 broadcaster
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.loop_hz = 1
        self.rate = rospy.Rate( self.loop_hz )

        self.pub = rospy.Publisher( w_top.body_ground_transform, TransformStamped, queue_size=1 )

        self.stand_up()
        self.stay_standing()

        # self.pose_control()

    #======================================================
    def stand_up( self ):

        # Initialize the robot to start on the ground
        self.pose.header.stamp = rospy.Time.now()
        self.pose.transform.translation.x = 0.0
        self.pose.transform.translation.y = 0.0
        self.pose.transform.translation.z = 0.0

        self.pose.transform.rotation.x = 0.0
        self.pose.transform.rotation.y = 0.0
        self.pose.transform.rotation.z = 0.0
        self.pose.transform.rotation.w = 1.0

        desired_speed = 0.05 # m/s

        t = time.time()
        t_prev = t

        while not rospy.is_shutdown() and self.pose.transform.translation.z < gt.body_height:
            t = time.time()
            dt = t - t_prev
            t_prev = t

            # Move the robot vertically
            self.pose.header.stamp = rospy.Time.now()
            self.pose.transform.translation.z += np.min( [desired_speed * dt, gt.body_height] )

            self.tf2_broadcaster.sendTransform( self.pose )

            self.pub.publish( self.pose )
            self.rate.sleep()

    #======================================================
    def stay_standing( self ):

        # Maintain old height, don't move it
        while not rospy.is_shutdown():
            self.pose.header.stamp = rospy.Time.now()

            self.tf2_broadcaster.sendTransform( self.pose )

            self.pub.publish( self.pose )
            self.rate.sleep()

    # #======================================================
    def pose_control( self ):

        self.stand_up()

        desired_speed = .01 # m/s

        t = time.time()
        t_prev = t

        while not rospy.is_shutdown() and self.pose.transform.translation.y < 0.035:

            t = time.time()
            dt = t - t_prev
            t_prev = t

            # Shift the robot forward
            # Move the robot vertically
            self.pose.header.stamp = rospy.Time.now()
            self.pose.transform.translation.y += desired_speed * dt

            print(f"y = {self.pose.transform.translation.y}")

            self.pub.publish( self.pose )
            self.rate.sleep()

        desired_speed = 0.03

        while not rospy.is_shutdown() and self.pose.transform.translation.y > -0.035:

            t = time.time()
            dt = t - t_prev
            t_prev = t

            # Shift the robot forward
            # Move the robot vertically
            self.pose.header.stamp = rospy.Time.now()
            self.pose.transform.translation.y -= desired_speed * dt

            print(f"y = {self.pose.transform.translation.y}")

            self.pub.publish( self.pose )
            self.rate.sleep()

        while not rospy.is_shutdown() and abs( self.pose.transform.translation.y ) > 0.002:

            t = time.time()
            dt = t - t_prev
            t_prev = t

            # Shift the robot forward
            # Move the robot vertically
            self.pose.header.stamp = rospy.Time.now()
            self.pose.transform.translation.y += desired_speed * dt

            print(f"y = {self.pose.transform.translation.y}")

            self.pub.publish( self.pose )
            self.rate.sleep()

#==============================================================================
if __name__ == "__main__":
    try:
        pc = pose_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
