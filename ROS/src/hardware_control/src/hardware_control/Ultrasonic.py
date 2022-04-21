#!/usr/bin/env python3
import time
import numpy as np
from hardware_control import hardware_constants
hrd = hardware_constants.hardware_constants()
if hrd.wanda:
    import RPi.GPIO as GPIO

#==============================================================================
class Ultrasonic:
    def __init__(self):
        self.trigger_pin = 27 # transmit
        self.echo_pin = 22 # receive
        self.half_sonic_speed = 344 / 2 # (Speed of sound in m/s) / (tx + rx)
        self.four_meter_time = 4 / self.half_sonic_speed # Datasheet says max distance should be 4m, so limit us to 6m time
        self.calibrated_offset = 0.022 # meters
        self.pulse_time = 0.00015 # seconds

        self.trigger_state = False
        self.echo_state = False

        if hrd.wanda:
            # Setup GPIO pins for the sensor only on hardware
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)

        self.set_transmit_state( False ) # Default to not pulsing signal

    #======================================================
    def set_transmit_state( self, state ):
        """ Sets pin state. Only sends command to pin if on hardware
        """
        self.trigger_state = state
        if hrd.wanda:
            # Only try to set the pin if running on hardware
            GPIO.output(self.trigger_pin, self.trigger_state)

    #======================================================
    def send_trigger_pulse(self):
        """ Pulses the sonic module for 0.00015s to send out 40kHz signal """
        self.set_transmit_state( True ) # Send signal
        time.sleep( self.pulse_time )
        self.set_transmit_state( False ) # Stop sending signal

    #======================================================
    def echo_return_time(self, pin_state, timeout):
        """
        Waits for state of pin to change to find time
        Args:
            pin_state ([type]): [description]
            timeout ([type]): [description]
        Returns:
            [type]: [description]
        """
        start_time = time.time()
        current_time = time.time()

        if hrd.wanda:
            self.echo_state = GPIO.input( self.echo_pin )

        while ( self.echo_state != pin_state ) and ( current_time - start_time < timeout ):
            if hrd.wanda:
                self.echo_state = GPIO.input( self.echo_pin )
            current_time = time.time()

        return current_time, self.echo_state

    #======================================================
    def get_distance(self):
        """ Pulses HC-SR04 sensor 3x and chooses the average distance """

        self.send_trigger_pulse()

        # Wait for the sensor to start receiving signal back
        start_time, returned = self.echo_return_time( True, self.four_meter_time )

        if returned:
            # If we got signal back, loop until we lose signal again
            finish_time, stopped = self.echo_return_time( False, self.four_meter_time )

            range = ( finish_time - start_time ) * self.half_sonic_speed + self.calibrated_offset
            return range, returned

        else:
            # Did not get signal back in time. Return nothing
            return -np.inf, returned

    #======================================================
    def average_distances( self, num_pulses ):
        """
        Pulses the sensor num_pulses times, returns the average and std dev range
        Args:
            num_pulses (int): Number of times to send and receive pulses
        Returns:
            mean (float): Average distance recorded. If no range found, returns 0.0
            std_dev (float): Standard deviation of ranges measured. If no range found, returns 0.0
        """
        distances = []
        ever_returned = False

        # Pulse the specified number of times
        for pulse in range( num_pulses ):

            distance, returned = self.get_distance()

            # Only store if we think we found something
            if returned:
                ever_returned = True
                distances.append( distance )

        if ever_returned:
            return np.mean( distances ), np.std( distances )
        else:
            return -np.inf, -np.inf


#==============================================================================
if __name__ == '__main__':
    sonic=Ultrasonic()
    while True:
        range, stddev= sonic.average_distances( 5 )
        print("Range, std: ", range, stddev)

