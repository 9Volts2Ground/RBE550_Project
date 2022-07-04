#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Imu

# Custom libraries
from hardware_constants import hardware_constants
from hw_topics import hw_topics

#----------------------------------------
# Set global flags and class instances
hrd = hardware_constants()
hw_top = hw_topics()

#==============================================================================
class IMU_hardware:

    # Make this class a singleton, so we only turn on the PWM channels once
    __instance = None
    def __new__(cls, *args, **kwargs):
        if not IMU_hardware.__instance:
            IMU_hardware.__instance = object.__new__(cls)
        return IMU_hardware.__instance

    #==========================================================================
    def __init__(self, wanda = False):
        self.wanda = wanda
        if self.wanda:
            # Set up PWM channel
            from mpu6050 import mpu6050 # https://github.com/m-rtijn/mpu6050
            self.sensor = mpu6050( address=0x68, bus=1 )
            self.sensor.set_accel_range( mpu6050.ACCEL_RANGE_2G ) # Acc can be +/- 2, 4, 8, or 16 g
            self.sensor.set_gyro_range( mpu6050.GYRO_RANGE_250DEG ) # Gyro can be +/- 250, 500, 1k, or 2k deg/s

        self.accel_measured = np.zeros( 3 )
        self.gyro_measured = np.zeros( 3 )

    #==========================================================================
    def get_current_measurement( self ):

        if self.wanda:
            try:
                accel_data = self.sensor.get_accel_data()
                gyro_data = self.sensor.get_gyro_data()

                # ToDo: verify this mapping for NWU coordinate frame
                self.accel_measured = np.array( [ accel_data['y'], -accel_data['x'], accel_data['z'] ] )
                self.gyro_measured  = np.array( [ gyro_data['x'], -gyro_data['z'], gyro_data['y'] ] ) * np.pi/180
            except:
                print("Can't update IMU data")

        else:
            # When not testing on hardware, return random values
            self.accel_measured = 2 * np.random.rand(3) - 1
            self.accel_measured[2] = 9.8 + 2 * np.random.rand(1) - 1 # Let's pretend there's some gravity here
            # self.accel_measured = self.accel_measured / np.linalg.norm( self.accel_measured )

            self.gyro_measured = np.random.rand(3) * np.pi/180

#==============================================================================
def imu_node():
    rospy.init_node( "imu_node", anonymous=True )

    pub = rospy.Publisher( hw_top.imu_data, Imu, queue_size=1 )

    rate = rospy.Rate( 100 ) # Hz, double check hardware spec

    # Initialize IMU topic message
    imu_topic = Imu( )

    # Initialize IMU hardware class
    imu_hrd = IMU_hardware( hrd.wanda )

    # While ROS is still running.
    while not rospy.is_shutdown():

        # Grab hardware data
        imu_hrd.get_current_measurement()

        # Store measured data into topic
        imu_topic.linear_acceleration.x = imu_hrd.accel_measured[0]
        imu_topic.linear_acceleration.y = imu_hrd.accel_measured[1]
        imu_topic.linear_acceleration.z = imu_hrd.accel_measured[2]

        imu_topic.angular_velocity.x = imu_hrd.gyro_measured[0]
        imu_topic.angular_velocity.y = imu_hrd.gyro_measured[1]
        imu_topic.angular_velocity.z = imu_hrd.gyro_measured[2]

        imu_topic.header.stamp = rospy.Time.now()
        pub.publish( imu_topic )

        rate.sleep()

#==============================================================================
if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        pass
