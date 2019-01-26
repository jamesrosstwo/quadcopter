import json
from tts import Speech
from watson import Watson

with open('config.json') as f:
    config = json.load(f)

assistant = Watson()
tts = Speech()

for i in range(20):
    message = assistant.message(input())
    print(message)
    tts.say(message)
