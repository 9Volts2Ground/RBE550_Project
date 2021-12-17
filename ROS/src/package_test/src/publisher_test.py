#!/usr/bin/env python3
import time

import rospy
# from std_msgs.msg import String
from package_test.msg import test

def publisher():
    # Publisher arguments:
    # Channel
    # Message type
    # queue_size
    pub = rospy.Publisher( "channel1", test, queue_size = 10)

    rospy.init_node( "publisher", anonymous=True)

    rate = rospy.Rate(1) # 1Hz
    t_start = time.time()

    t_msg = test() # Instantiating the message type
    t_msg.message_num = 0

    while not rospy.is_shutdown():
        now = time.time() - t_start

        t_msg.current_time = "Running time: " + str(now)
        t_msg.message_num += 1

        pub.publish( t_msg )

        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    




