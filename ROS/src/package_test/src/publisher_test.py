#!/usr/bin/env python3
import time

import rospy
from std_msgs.msg import String

def publisher():
    # Publisher arguments:
    # Channel
    # Message type
    # queue_size
    pub = rospy.Publisher( "channel1", String, queue_size = 10)

    rospy.init_node( "publisher", anonymous=True)

    rate = rospy.Rate(10) # 10Hz
    t_start = time.time()

    while not rospy.is_shutdown():
        now = time.time() - t_start

        pub.publish( "Running time: " + str(now) )

        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    




