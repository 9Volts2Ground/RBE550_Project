#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming
# video from your built-in webcam
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
top = hw_topics()

def camera_image_node():

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('camera_image_node', anonymous=True)

    # Node is publishing to the video_frames topic using the message type Image
    pub = rospy.Publisher(top.camera_image, Image, queue_size=10)

    # Control max rate of image capture, Hz
    rate = rospy.Rate( 10 )

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    cap = cv2.VideoCapture( 0 )

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    image_encoding = "bgr8"

    # While ROS is still running.
    while not rospy.is_shutdown():

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        image_captured, image = cap.read()

        if image_captured == True:
            # Print debugging information to the terminal
            rospy.loginfo( 'publishing video frame' )

            ( rows, cols, channels ) = image.shape

            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS image message
            image_message = br.cv2_to_imgmsg( image, encoding=image_encoding )
            image_message.header.stamp = rospy.Time.now()
            image_message.encoding = image_encoding
            image_message.height = rows # Include size of the image
            image_message.width = cols
            pub.publish( image_message )

        else:
            rospy.loginfo( 'Could not capture image frame' )

        # Sleep just enough to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_image_node()
    except rospy.ROSInterruptException:
        pass