import datetime
import json
import time


def get_readings():
    with open("readings.json") as f:
        return f.read()


class Data:
    def __init__(self):
        self.status = "on"
        self.axes = []

    def get_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

    def get_list(self):
        return json.loads(self.get_json())

    def write_to(self, filename):
        with open(filename, "w+") as f:
            f.write(self.get_json())
            f.write(chr(3))
            f.close()


if __name__ == "__main__":
    d = Data()
    print(d.get_json())
