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
deg2rad = np.pi/180
chn = channels.channels()
hrd = hardware_constants.hardware_constants() # Class with

wanda = "wanda" in os.getcwd()
if wanda:
    # Only initialize hardware class if we are running on Wanda hardware
    from Servo import Servo
    servo = Servo()

#==============================================================================
def move_all_joints( leg_states ):
    """ Command all motors to move to joint angles defined in joint_angles """

    joint_angles = np.reshape( leg_states.joint_angles, (3,6))

    # Loop through all legs and joints
    for leg in range(6):
        for joint in range(3):

            # Convert angle to degrees, offset it by the hardware 0 value
            angle_command = joint_angles[joint,leg] * deg2rad * hrd.motorOrientation[joint,leg] + hrd.motorCenter[joint,leg]

            # Saturate the motor commands to min/max values
            if angle_command > hrd.motorMax[joint,leg]:
                angle_command = hrd.motorMax[joint,leg]
            elif angle_command < hrd.motorMin[joint,leg]:
                angle_command = hrd.motorMin[joint,leg]

            # Ensure angle is an int to pass into setServoAngle()
            angle_command = int( angle_command )

            # Only try to move the motors and set LED if we are running on real hardware
            if wanda:
                servo.setServoAngle( hrd.legChannel[joint, leg], angle_command )

#==============================================================================
def move_motors_node():
    rospy.init_node( "move_motors_node", anonymous=True )
    rospy.Subscriber( chn.leg_states, leg_states, move_all_joints )

    rospy.spin()


#==============================================================================
if __name__ == '__main__':
    move_motors_node()