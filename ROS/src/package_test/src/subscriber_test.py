#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def terminal_print(msg):
    print("New message: ", msg.data)

def subscriber():
    
    rospy.init_node( "subscriber", anonymous=True)
    rospy.Subscriber("channel2", String, terminal_print)

    rospy.spin()

if __name__ == '__main__':
    subscriber()

