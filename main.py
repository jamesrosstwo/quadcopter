import json
from text_to_speech import Speech
from speech_to_text import Recognizer
from watson import Watson
from server import Server
import datetime


def load_config():
    with open('config.json') as f:
        return json.load(f)


def init_services():
    global assistant, tts, recognizer, web_server
    print("Initializing conversation...")
    assistant = Watson()
    print("Initializing voice...")
    tts = Speech()
    print("Initializing speech recognition...")
    recognizer = Recognizer()

    # maybe just store this in memory?
    with open('config.json') as f:
        data = json.load(f)
        print("Initializing web server...")
        web_server = Server(data)


def update_data(update, delay_ms):
    td = (update - datetime.datetime.now()).total_seconds() * 1000
    if td < 0:
        return datetime.datetime.now() + datetime.timedelta(milliseconds=delay_ms)
    return update



if __name__ == "__main__":
    data_cooldown = 500
    next_update = datetime.datetime.min
    config = load_config()
    init_services()

    print("Watson:", assistant.message(""))
    while True:
        next_update = update_data(next_update, data_cooldown)

    # if buttonPressed:
    # message = assistant.message(recognizer.listen())
    # print("Watson:", message)
    # tts.say(message)
