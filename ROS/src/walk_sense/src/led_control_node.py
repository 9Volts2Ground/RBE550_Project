#!/usr/bin/env python3
import numpy as np
import os
import rospy

# Custom libraries
import channels
import hardware_constants
from walk_sense.msg import leg_states

#----------------------------------------
# Set globla flags and class instances
chn = channels.channels()
hrd = hardware_constants.hardware_constants()

wanda = "wanda" in os.getcwd()
if wanda:
    # Only initialize LED array if running on hardware
    from Led import Led
    led = Led()

#==============================================================================
def set_leg_led( leg_states ):
    for leg in range( 6 ):
        if leg_states.foot_off_ground[leg]:
            led_color = [0, 0, 10]
        else:
            led_color = [0, 10, 0]

        if wanda:
            try:
                led.setColor( hrd.leg2led[leg], led_color )
            except:
                pass

#==============================================================================
def led_control_node():
    rospy.init_node( "led_control_node", anonymous=True )
    rospy.Subscriber( chn.leg_states, leg_states, set_leg_led )
    rospy.spin()

#==============================================================================
if __name__ == '__main__':
    led_control_node()
