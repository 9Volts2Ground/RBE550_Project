#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image # Image is the message type

# Custom libraries
from classes import walk_topics
from hardware_control.hw_topics import hw_topics # List of acceptable channel names


#----------------------------------------
# Set global flags and class instances
hw_top = hw_topics()
w_top = walk_topics.walk_topics()

#==============================================================================
class camera_image_target_processing():
    def __init__(self):

        # Initialize ROS communication
        rospy.init_node( "camera_image_target_processing", anonymous=True)

        rospy.Subscriber(hw_top.camera_image, Image, self.process_image)

        self.pub = rospy.Publisher( "channel2", test, queue_size = 10)


    #==========================================================================
    def process_image(self, tst):

        self.pub.publish( tst )


#==============================================================================
if __name__ == '__main__':

    try:
        ps = camera_image_target_processing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

