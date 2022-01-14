import copy
import math
import numpy as np
import os
import time

from rotation import rotrx, rotry, rotrz

class IMU:

    # Make this class a singleton, so we only turn on the PWM channels once
    __instance = None
    def __new__(cls, *args, **kwargs):
        if not IMU.__instance:
            IMU.__instance = object.__new__(cls)
        return IMU.__instance

    #==========================================================================
    def __init__(self, wanda = False):

        self.wanda = wanda
        if self.wanda:
            # Set up PWM channel
            from mpu6050 import mpu6050 # https://github.com/m-rtijn/mpu6050
            try:
                self.sensor = mpu6050( address=0x68, bus=1 )
                self.sensor.set_accel_range( mpu6050.ACCEL_RANGE_2G ) # Acc can be +/- 2, 4, 8, or 16 g
                self.sensor.set_gyro_range( mpu6050.GYRO_RANGE_250DEG ) # Gyro can be +/- 250, 500, 1k, or 2k deg/s
            except:
                print("Could not connect to IMU")

        # Current measurement values
        self.accel_measured = np.zeros(3) # m/s**2
        self.gyro_measured = np.zeros(3) # pitch, roll, yaw rates, rad/s

        # Track previous state
        self.accel_measured_prev = np.zeros(3)
        self.gyro_measured_prev = np.zeros(3)

        # Average between current and previous state
        self.accel_averaged = np.zeros(3)
        self.gyro_averaged = np.zeros(3)

        # Measurement times
        self.measured_time = time.time()
        self.previous_measured_time = self.measured_time

        self.filter_period = 1/50 # 20 Hz low-pass filter
        self.init_filter = False

        # Collect initial resting value of sensor data to set baseline error/offsets
        print("Measuring IMU gravity offsets. Please leave body static")
        self.accel_mean, self.gyro_mean, self.accel_std, self.gyro_std = self.average_filter(0.5)
        print("Finished calibrating IMU. Proceed with movement.")

        # Initialize low-pass filter values
        self.accel_filtered = self.accel_mean
        self.gyro_filtered = self.gyro_mean

        # Gravity
        # self.gravity = 9.79247 # Gravity constant for Tucson, m/s**2
        self.gravity_body = self.accel_mean
        self.gravity_mag = np.linalg.norm( self.accel_mean )

        # After taking initial poll, set initial Euler angles relative to world frame
        self.pitch = np.arctan2( self.gravity_body[1], np.sqrt(self.gravity_body[0]**2 + self.gravity_body[2]**2) )
        self.roll = np.arctan2( -self.gravity_body[0], self.gravity_body[2])
        self.yaw = 0.0
        self.update_euler_angles()

        # Convert gravity vector to inertial frame
        # self.gravity_inertial = rotry( self.roll ) @ rotrx( self.pitch ) @ rotrz( self.yaw ) @ self.gravity_body.T
        # self.gravity_inertial = rotry( self.roll ) @ rotrx( self.pitch ) @ self.gravity_body.T
        # Assume we are pointing straight up to start with
        self.gravity_inertial = self.gravity_body

    #==========================================================================
    def update_euler_angles( self ):
        """
        Sets Euler angle vector in radians. Yaw, Pitch, Roll
        """
        self.euler_angles = np.array( [self.yaw, self.pitch, self.roll] )

    #==========================================================================
    def get_current_measurement( self ):

        self.accel_measured_prev = copy.deepcopy( self.accel_measured )
        self.gyro_measured_prev = copy.deepcopy( self.gyro_measured )

        if self.wanda:
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()

            self.accel_measured = np.array( [ accel_data['x'], accel_data['y'], accel_data['z'] ] )
            self.gyro_measured  = np.array( [ gyro_data['z'],  gyro_data['x'],  gyro_data['y'] ] ) * np.pi/180

        else:
            # When not testing on hardware, return random values
            self.accel_measured = np.random.rand(3)
            self.accel_measured = self.accel_measured * 9.79247 / np.linalg.norm( self.accel_measured )
            self.gyro_measured = np.random.rand(3) * np.pi/180

        # Take average of current and previous measurements
        self.accel_averaged = np.mean([self.accel_measured, self.accel_measured_prev], axis=0)
        self.gyro_averaged = np.mean([self.gyro_measured, self.gyro_measured_prev], axis=0)

        # Update the time we polled the IMU sensor
        self.previous_measured_time = self.measured_time
        self.measured_time = time.time()

        if self.init_filter:
            self.low_pass_filter( dt = self.measured_time - self.previous_measured_time )


    #==========================================================================
    def average_filter( self, average_time ):
        """
        Takes average sensor measures, used to establish any baseline offsets
        Returns:
            accel, gyro: 1x3 np arrays
        """
        accel = np.zeros(3)
        gyro = np.zeros(3)

        t = time.time()
        t_end = t + average_time
        while t <= t_end:
            self.get_current_measurement() # Take new measurement of sensor states

            accel = np.vstack( [accel, self.accel_measured] )
            gyro = np.vstack( [gyro, self.gyro_measured] )
            t = time.time()

        accel_mean = np.array( [np.mean(accel[1:,0]), np.mean(accel[1:,1]) , np.mean(accel[1:,2])] )
        accel_std = np.array( [np.std(accel[1:,0]), np.std(accel[1:,1]) , np.std(accel[1:,2])] )
        # accel[2] = accel[2] - self.gravity # Remove gravity offset, gotta refine this

        gyro_mean = np.array( [np.mean(gyro[1:,0]), np.mean(gyro[1:,1]) , np.mean(gyro[1:,2]) ] )
        gyro_std = np.array( [np.std(gyro[1:,0]), np.std(gyro[1:,1]) , np.std(gyro[1:,2]) ] )

        self.init_filter = True

        return accel_mean, gyro_mean, accel_std, gyro_std

    #==========================================================================
    def low_pass_filter( self, dt = 0.01 ):
        alpha = dt / ( self.filter_period + dt )

        self.accel_filtered = self.accel_filtered + alpha * (self.accel_averaged - self.accel_filtered )
        self.gyro_filtered = self.gyro_filtered + alpha * ( self.gyro_averaged - self.gyro_filtered )


#==========================================================================
if __name__ == '__main__':
    s=IMU()
    time1=time.time()
    while True:
        try:
            time.sleep(0.01)
            r,p,y=s.imuUpdate()
            print(r,p,y)
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
