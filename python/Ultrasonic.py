import time
import numpy as np
import RPi.GPIO as GPIO
class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)

        GPIO.output(self.trigger_pin,False) # Default to not pulsing signal

        self.half_sonic_speed = 344 / 2 # (Speed of sound in m/s) / (tx + rx)
        self.six_meter_time = (4 + 2) * 2 / 344 # Datasheet says max distance should be 4m, so limit us to 6m time
        self.calibrated_offset = .022 # m

    #--------------------------------------------------------------------------
    def send_trigger_pulse(self):
        """ Pulses the sonic module for 0.00015s to send out 40kHz signal """
        GPIO.output( self.trigger_pin, True ) # Send signal
        time.sleep(0.00015)
        GPIO.output( self.trigger_pin, False ) # Stop sending signal

    #--------------------------------------------------------------------------
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

        while GPIO.input(self.echo_pin) != pin_state and current_time - start_time < timeout:
            current_time = time.time()

        return current_time, GPIO.input(self.echo_pin)

    #--------------------------------------------------------------------------
    def get_distance(self):
        """ Pulses HC-SR04 sensor 3x and chooses the average distance """

        self.send_trigger_pulse()

        # Wait for the sensor to start receiving signal back
        start, returned = self.echo_return_time( True, self.six_meter_time )

        if returned:
            # If we got signal back, loop until we lose signal again
            finish, stopped = self.echo_return_time( False, self.six_meter_time )

            range = self.calibrated_offset + ( finish - start ) * self.half_sonic_speed
            return range, returned
        
        else:
            # Did not get signal back in time. Return nothing
            return 0.0, returned

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
            return 0.0, 0.0


# Main program logic follows:
if __name__ == '__main__':
    sonic=Ultrasonic()
    while True:
        range, stddev= sonic.average_distances( 5 )
        print("Range, std: ", range, stddev)

