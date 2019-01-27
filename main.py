import json
from text_to_speech import Speech
from speech_to_text import Recognizer
from watson import Watson

if __name__ == "__main__":
    with open('config.json') as f:
        config = json.load(f)

    print("Initializing Watson...")
    assistant = Watson()
    print("Initializing voice...")
    tts = Speech()
    print("Initializing speech recognition")
    recognizer = Recognizer()

    print("Speak now")
    for i in range(20):
        message = assistant.message(recognizer.listen())
        print(message)
        tts.say(message)
