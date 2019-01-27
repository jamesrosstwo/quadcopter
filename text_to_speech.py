import pyttsx3


class Speech:
    def __init__(self):
        self.engine = pyttsx3.init()

    def say(self, message):
        self.engine.say(message)
        self.engine.runAndWait()


if __name__ == "__main__":
    print("")