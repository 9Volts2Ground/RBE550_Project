#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import imutils
import numpy as np
import rospy
from sensor_msgs.msg import Image # Image is the message type

# Custom libraries
from classes import walk_topics
from hardware_control.hw_topics import hw_topics # List of acceptable channel names
from walk_sense.msg import target_states

#----------------------------------------
# Set global flags and class instances
hw_top = hw_topics()
w_top = walk_topics.walk_topics()

detection_radius = 50

#==============================================================================
class color():
    ''' Class for containing HSV color ranges
    '''
    def __init__( self, low=[], high=[], color='' ):
        self.high = high
        self.low = low
        self.color = color


# define the color ranges
green = color( low = np.array([29, 86, 6]), high = np.array([64, 255, 255]), color='green' )
blue = color( low = np.array([57, 68, 0]), high=np.array([151, 255, 255]), color='blue')

#==============================================================================
class camera_image_target_processing():
    def __init__(self):

        # Initialize ROS communication
        rospy.init_node( "camera_image_target_processing", anonymous=True)

        self.pub = rospy.Publisher( w_top.target_states, target_states, queue_size = 10)

        self.br = CvBridge() # Converts image ROS topics to cv2 objects

        rospy.Subscriber(hw_top.camera_image, Image, self.process_image)


    #==========================================================================
    def process_image(self, image_topic):

        color = blue

        # Convert camera data to a cv2 object
        frame = self.br.imgmsg_to_cv2( image_topic )

        # Initialize target state message
        tgt_state = target_states()
        tgt_state.header.stamp = rospy.Time.now()
        tgt_state.camera_width = image_topic.width
        tgt_state.camera_height = image_topic.height

        # Blur the image, convert it to the HSV color space
        blurred = cv2.GaussianBlur(frame, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange( hsv, color.low, color.high) # Grab initial filter for desired color
        mask = cv2.erode( mask, None, iterations=2 ) # First pass of filtering
        mask = cv2.dilate( mask, None, iterations=2 )

        # find contours in the mask
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # only proceed if at least one contour was found
        if np.size( contours ):

            # find the largest contour in the mask, then use it to compute
            # the minimum enclosing circle and centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # only draw the enclosing circle and text if the radious meets
            # a minimum size
            if radius >= detection_radius:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

                M = cv2.moments(c)
                (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.putText(frame, color.color, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

                tgt_state.target_found = True
                tgt_state.target_position.x = cX
                tgt_state.target_position.y = cY

        # Display data, if we want
        cv2.imshow( "target_states", frame)
        cv2.waitKey(1)

        # Publish simple topic with detected target state info
        self.pub.publish( tgt_state )


#==============================================================================
if __name__ == '__main__':

    try:
        ps = camera_image_target_processing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

