#!/usr/bin/env python3
import numpy as np
import os
import rospy

# Custom libraries
from classes import channels
from classes import hardware_constants
from classes import Servo
from walk_sense.msg import leg_states

#----------------------------------------
# Set global flags and class instances
chn = channels.channels()
hrd = hardware_constants.hardware_constants() # Class with
servo = Servo.Servo()

rad2deg = 180/np.pi
num_legs = 6
num_joints = 3 # 3 DoF legs

#==============================================================================
def move_leg_motors( leg_states ):

    for joint in range(num_joints):

        # Convert angle to degrees, offset it by the hardware 0 value
        angle_command = leg_states.joint_angle.position[joint] * rad2deg * hrd.motorOrientation[joint,leg_states.leg_num] + hrd.motorCenter[joint,leg_states.leg_num] # Offset calculation based on center of calibrated servo

        # Saturate the motor commands to min/max values
        if angle_command > hrd.motorMax[joint,leg_states.leg_num]:
            angle_command = hrd.motorMax[joint,leg_states.leg_num]
        elif angle_command < hrd.motorMin[joint,leg_states.leg_num]:
            angle_command = hrd.motorMin[joint,leg_states.leg_num]

        # Ensure angle is an int to pass into setServoAngle()
        angle_command = int( angle_command )

        try:
            servo.setServoAngle( hrd.legChannel[joint, leg_states.leg_num], angle_command )
        except:
            print("Could not move servo: ", leg_states.leg_num, joint)


#==============================================================================
def move_motors_node():
    rospy.init_node( "move_motors_node", anonymous=True )

    # Kick off unique subscriber for each leg
    for leg in range( num_legs ):
        rospy.Subscriber( chn.leg_states[leg], leg_states, move_leg_motors )

    rospy.spin()

#==============================================================================
if __name__ == '__main__':
    move_motors_node()