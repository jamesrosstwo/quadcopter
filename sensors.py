from ping import PingSensor
from gyroscope import Gyroscope
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
ping = PingSensor(5, 6)
gyroscope = Gyroscope(1)
