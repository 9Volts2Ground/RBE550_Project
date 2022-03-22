#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

# Custom libraries
from hw_topics import hw_topics

#----------------------------------------
# Set global flags and class instances
top = hw_topics() # ROS topics to subscribe to
br = CvBridge() # Used to convert between ROS and OpenCV images

#==============================================================================
def camera_image_display_node():

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('camera_image_display_node', anonymous=True)

    # Node is subscribing to the camera image topic
    rospy.Subscriber(top.camera_image, Image, display_image)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()
    
#==============================================================================
def display_image(data):

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2( data )

    # Display image
    cv2.imshow( "camera", current_frame )

    cv2.waitKey(1)

#==============================================================================
if __name__ == '__main__':
    camera_image_display_node()