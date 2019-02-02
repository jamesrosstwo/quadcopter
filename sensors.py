from ping import PingSensor
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
ping = PingSensor(5, 6)
