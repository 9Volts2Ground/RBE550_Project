#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class pub_sub():
    def __init__(self):
        rospy.init_node( "pub_sub", anonymous=True)

        rospy.Subscriber("channel1", String, self.terminal_print)

        self.pub = rospy.Publisher( "channel2", String, queue_size = 10)


    def terminal_print(self, msg):

        self.pub.publish( "Forwarding: " + msg.data)



if __name__ == '__main__':

    ps = pub_sub()

    rospy.spin()

