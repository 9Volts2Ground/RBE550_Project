#!/usr/bin/env python3
import rospy
from package_test.msg import test
# from std_msgs.msg import String

class pub_sub():
    def __init__(self):

        rospy.init_node( "pub_sub", anonymous=True)

        rospy.Subscriber("channel1", test, self.terminal_print)

        self.pub = rospy.Publisher( "channel2", test, queue_size = 10)


    def terminal_print(self, tst):

        self.pub.publish( tst )



if __name__ == '__main__':

    ps = pub_sub()

    rospy.spin()

