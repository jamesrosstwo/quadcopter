import threading
from server import *
from data import *
import RPi.GPIO as GPIO


def load_config():
    with open('config.json') as f:
        return json.load(f)


if __name__ == "__main__":
    config = load_config()
    server = Server()
    data = Data()
    server.check(data)
