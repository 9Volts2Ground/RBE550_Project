#!/usr/bin/env python3
import numpy as np
import rospy

# Custom libraries
from hw_topics import hw_topics
from hardware_constants import hardware_constants
from Servo import Servo
from hardware_control.msg import leg_states, seeker_states

#----------------------------------------
# Set global flags and class instances
top = hw_topics()
hrd = hardware_constants()
servo = Servo()

seeker_motors = ['az', 'el']

rad2deg = 180/np.pi

#==============================================================================
def move_leg_motors( leg_states ):

    for joint in range(hrd.num_joints):

        # Convert angle to degrees, offset it by the hardware 0 value
        # Offset calculation based on center of calibrated servo
        angle_command = leg_states.joint_angle.position[joint] * rad2deg * hrd.motorOrientation[joint,leg_states.leg_num] + hrd.motorCenter[joint,leg_states.leg_num]

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
def move_seeker_motors( seeker_states ):

    for joint in range( hrd.num_seeker_joints ):

        # Convert angle to degrees, offset it by the hardware 0 value
        # Offset calculation based on center of calibrated servo
        angle_command = seeker_states.joint_angle.position[joint] * rad2deg * hrd.seekerOrientation[joint] + hrd.seekerCenter[joint]

        # Saturate the motor commands to min/max values
        if angle_command > hrd.seekerMax[joint]:
            angle_command = hrd.seekerMax[joint]
        elif angle_command < hrd.seekerMin[joint]:
            angle_command = hrd.seekerMin[joint]

        # Ensure angle is an int to pass into setServoAngle()
        angle_command = int( angle_command )

        try:
            servo.setServoAngle( hrd.seeker_channel[joint], angle_command )
        except:
            print("Could not move seeker servo: ", seeker_motors[joint])


#==============================================================================
def move_motors_node():
    rospy.init_node( "move_motors_node", anonymous=True )

    # Kick off subscriber for the seeker
    rospy.Subscriber( top.seeker_states, seeker_states, move_seeker_motors )

    # Kick off unique subscriber for each leg
    for leg in range( hrd.num_legs ):
        rospy.Subscriber( top.leg_states[leg], leg_states, move_leg_motors )

    rospy.spin()

#==============================================================================
if __name__ == '__main__':
    move_motors_node()