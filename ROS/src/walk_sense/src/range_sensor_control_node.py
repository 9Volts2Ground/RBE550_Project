#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy

# Custom classes
from classes import walk_topics
from hardware_control import Ultrasonic
from sensor_msgs.msg import Range
from walk_sense.msg import target_track

w_top = walk_topics.walk_topics()

class range_sensor_control_node():
    def __init__(self):
        rospy.init_node( "range_sensor_control_node", anonymous=True )

        # Initialize ultrasonic range sensor class
        self.sonic = Ultrasonic.Ultrasonic()

        # Initialize state topics
        self.target_track = target_track()

        self.range_sensor = Range()
        self.range_sensor.header.stamp = rospy.Time.now()
        self.range_sensor.radiation_type = self.range_sensor.ULTRASOUND
        self.range_sensor.min_range = 0.02 # meters
        self.range_sensor.max_range = 4 # meters
        self.range_sensor.field_of_view = 15 * np.pi/180

        self.pub = rospy.Publisher( w_top.range_sensor, Range, queue_size=1 )

        rospy.Subscriber( w_top.target_track, target_track, self.grab_target_track )

        self.range_sensor_control()

    #======================================================
    def range_sensor_control(self):
        rate = rospy.Rate( 10 )

        print("Range sensor loop...")

        while not rospy.is_shutdown():
            target_track = copy.deepcopy( self.target_track )

            if target_track.tracking_state in ['target_track', 'target_lost']:
                # Ping for range measurement, publish data
                range, returned = self.sonic.get_distance()

                self.range_sensor.range = range

                self.pub.publish( self.range_sensor )

            rate.sleep()


    #======================================================
    def grab_target_track(self, target_track):

        self.target_track = copy.deepcopy( target_track )

#==============================================================================
if __name__ == "__main__":
    try:
        rs = range_sensor_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

