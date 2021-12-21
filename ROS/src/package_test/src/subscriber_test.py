#!/usr/bin/env python3
import rospy
# from std_msgs.msg import String
from package_test.msg import test

def terminal_print(msg):
    print("New message: ", msg)

def subscriber():

    rospy.init_node( "subscriber", anonymous=True)
    rospy.Subscriber("channel2", test, terminal_print)

    rospy.spin()

if __name__ == '__main__':
    subscriber()

