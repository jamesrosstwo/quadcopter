from socket import *
from data import *

class Server:
    def __init__(self):
        self.open = True
        self.HOST = 'localhost'
        self.PORT = 50000
        self.BUFFER_SIZE = 1024
        self.ADDRESS = (self.HOST, self.PORT)
        self.server = socket(AF_INET, SOCK_STREAM)
        self.server.bind(self.ADDRESS)
        self.server.listen(2)

    def check(self):
        while True:
            print("waiting on connection")
            client, address = self.server.accept()
            print('connected from:', address)
            while 1:
                client_response = client.recv(1024).decode()
                if not client_response:
                    break
                if client_response == "readings":
                    client_response = get_readings()
                client.send(client_response.encode())
            client.close()

    def close(self):
        self.open = False
        self.server.close()


if __name__ == "__main__":
    s = Server()
    d = Data()
    readings = d.get_json()
    s.check()
    s.close()