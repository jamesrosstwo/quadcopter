import speech_recognition as sr


class Recognizer:
    def __init__(self):
        self.microphone = sr.Microphone()
        self.recognizer = sr.Recognizer()

    def listen(self):
        with self.microphone as source:
            audio = self.recognizer.listen(source)
        try:
            return self.recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return ""


if __name__ == "__main__":
    r = Recognizer()
    for i in range(10):
        print(r.listen())
