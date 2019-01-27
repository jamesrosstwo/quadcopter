import json
from text_to_speech import Speech
from speech_to_text import Recognizer
from watson import Watson

if __name__ == "__main__":
    with open('config.json') as f:
        config = json.load(f)

    print("Initializing conversation...")
    assistant = Watson()
    print("Initializing voice...")
    tts = Speech()
    print("Initializing speech recognition")
    recognizer = Recognizer()

    print("Watson:", assistant.message(""))
    for i in range(20):
        message = assistant.message(recognizer.listen())
        print("Watson:", message)
        tts.say(message)
