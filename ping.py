import time

import RPi.GPIO as GPIO

class PingSensor:
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

    def read(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)
        start_time = time.time()
        stop_time = time.time()
        while GPIO.input(self.echo) == 0:
            start_time = time.time()
        while GPIO.input(self.echo) == 1:
            stop_time = time.time()
        TimeElapsed = stop_time - start_time
        distance = (TimeElapsed * 34300) / 2
        return distance