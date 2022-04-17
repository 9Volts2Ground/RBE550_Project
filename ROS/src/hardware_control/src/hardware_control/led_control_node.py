#!/usr/bin/env python3
import rospy

# Custom libraries
from hw_topics import hw_topics
from Led import Led
from hardware_constants import hardware_constants
from hardware_control.msg import leg_states

#==============================================================================
class colors():
    def __init__(self):
        self.red =      [10, 0, 0]
        self.green =    [0, 0, 10]
        self.blue =     [0, 10, 0]

#----------------------------------------
# Set global flags and class instances
hw_top = hw_topics()
hrd = hardware_constants()
led = Led()
color = colors()

#==============================================================================
def led_control_node():
    rospy.init_node( "led_control_node", anonymous=True )

    for leg in range( hrd.num_legs ):
        rospy.Subscriber( hw_top.leg_states[leg], leg_states, set_leg_led )

    rospy.spin()

#==============================================================================
def set_leg_led( leg_states ):

    if leg_states.foot_off_ground:
        led_color = color.green
    else:
        led_color = color.blue

    try:
        led.setColor( hrd.leg2led[leg_states.leg_num], led_color )
        print( led_color )
    except:
        pass
        # print( f"led_color {leg_states.leg_num} (not lit) = {led_color}")

#==============================================================================
if __name__ == '__main__':
    try:
        led_control_node()
    except rospy.ROSInterruptException:
        pass
