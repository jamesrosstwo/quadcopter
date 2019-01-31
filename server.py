import json
from socket import *


class Server:
    def get_readings(self):
        with open('readings.json') as f:
            return f.read()

    def update_readings(self, r):
        with open('readings.json', "w+") as f:
            f.write(json.dumps(r))
            f.write(chr(3))
            f.close()

    def __init__(self):
        self.HOST = 'localhost'
        self.PORT = 50000
        self.BUFFER_SIZE = 1024
        self.ADDRESS = (self.HOST, self.PORT)
        self.server = socket(AF_INET, SOCK_STREAM)
        self.server.bind(self.ADDRESS)
        self.server.listen(2)

    def start(self):
        while True:
            print("waiting on connection")
            client, address = self.server.accept()
            print('connected from:', address)
            while 1:
                data = client.recv(1024).decode()
                if not data:
                    break
                if data == "readings":
                    data = self.get_readings()
                client.send(data.encode())
            client.close()

        server.close()


if __name__ == "__main__":
    s = Server()
    readings = {"status": "on"}
    s.update_readings(readings)
    s.start()
