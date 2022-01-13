#!/usr/bin/env python3
import numpy as np
import os
import rospy
import time

# Custom libraries
import channels
import hardware_constants
from walk_sense.msg import leg_states

#----------------------------------------
# Set global flags and class instances
chn = channels.channels()
hrd = hardware_constants.hardware_constants()

wanda = hrd.wanda
if wanda:
    # Only initialize LED array if running on hardware
    from Led import Led
    led = Led()

class inertial_state_estimation():
    def __init__(self):
        
        rospy.init_node( "inertial_state_estimation", anonymous=True)
        rospy.Subscriber( chn.leg_states,  )





