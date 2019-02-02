import _thread
from server import *
from data import *
import RPi.GPIO as GPIO



def load_config():
    with open('config.json') as f:
        return json.load(f)


if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    config = load_config()
    server = Server()
    data = Data()
    _thread.start_new_thread(server.check(), ())
    while True:
        data.update()

