import threading
from server import *
from data import *
import RPi.GPIO as GPIO


def load_config():
    with open('config.json') as f:
        return json.load(f)


if __name__ == "__main__":
    print("config loading")
    config = load_config()
    print("config")
    server = Server()
    print("server up")
    data = Data()
    print("data up")
    server.check(data)
