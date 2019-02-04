import time
import RPi.GPIO as GPIO


class PingSensor:
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo
        print(trig, echo, type(trig), type(echo))
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        self.dist_size = 7
        r = self.read(first_reading=True)
        self.dists = [r / self.dist_size] * self.dist_size
        self.distance = r

    def update_dist(self, dist_reading):
        self.distance -= self.dists.pop(0)
        self.dists.append(dist_reading)
        self.distance += dist_reading
        return self.distance

    def read(self, first_reading=False):
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
        if first_reading:
            return int(distance)
        return int(self.update_dist(distance / self.dist_size))


if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    ping = PingSensor(5, 6)
    while True:
        print(ping.read())
