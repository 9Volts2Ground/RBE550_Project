#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming video
# Based on code from:
# - https://automaticaddison.com
# - https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/

# Import the necessary libraries
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import rospy
from sensor_msgs.msg import Image

# Custom libraries
from hardware_constants import hardware_constants
from hw_topics import hw_topics

#----------------------------------------
# Set global flags and class instances
hrd = hardware_constants()
top = hw_topics()

is_wanda = hrd.is_wanda()

def camera_image_node():

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('camera_image_node', anonymous=True)

    # Node is publishing to the video_frames topic using the message type Image
    pub = rospy.Publisher(top.camera_image, Image, queue_size=10)

    # Control max rate of image capture, Hz
    rate = rospy.Rate( 10 )

    # Create object to capture image
    if is_wanda:
        # Use RPi-specific camera libraries
        from picamera import PiCamera
        from picamra.array import PiRGBArray
        cam = PiCamera()
        resolution = ( 640, 480 )
        cam.resolution = resolution
        cam.framerate = 10
        cam_image = PiRGBArray( cam, resolution )
    else:
        # Use generic camera cv2 library
        cam = cv2.VideoCapture( 0 ) # The argument '0' gets the default webcam

    image_encoding = "bgr8"

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # While ROS is still running.
    while not rospy.is_shutdown():

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        if is_wanda:
            frame = cam.capture( cam_image, format='bgr', use_video_port=True )
            image = frame.array
            cam_image.truncate(0) # Clear video stream to prep for next frame
        else:
            image_captured, image = cam.read()

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