import datetime
import json
import sensors
import time


def get_readings():
    with open("readings.json") as f:
        return f.read()


class Data:

    def __init__(self):
        self.status = "on"
        self.altitude = sensors.ping.read()
        self.rotation_x = 0
        # self.rotation_y = 0
        # self.rotation_z = 0

    def update(self):
        self.altitude = sensors.ping.read()
        # self.rotation_x, self.rotation_y, self.rotation_z = sensors.gyroscope.read()
        self.rotation_x = sensors.gyroscope.read()[0]
        self.write_to("readings.json")
        return self.get_json()

    def get_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

    def write_to(self, filename):
        with open(filename, "w+") as f:
            f.write(self.get_json())
            f.write(chr(3))
            f.close()


if __name__ == "__main__":
    d = Data()
    print(d.get_json())
