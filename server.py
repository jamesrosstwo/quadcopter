from socket import *
import data
import sys


class Server:
    def __init__(self):
        self.open = True
        self.HOST = '0.0.0.0'
        self.PORT = int(sys.argv[1])
        self.BUFFER_SIZE = 1024
        self.ADDRESS = (self.HOST, self.PORT)
        self.server = socket(AF_INET, SOCK_STREAM)
        self.server.bind(self.ADDRESS)
        self.server.listen(2)

    def check(self, d):
        while True:
            print("waiting on connection")
            client, address = self.server.accept()
            print('connected from:', address)
            while 1:
                client_response = client.recv(1024).decode()
                print(client_response)
                if not client_response:
                    break
                if client_response == "readings":
                    print("kachow")
                    client_response = data.get_readings()
                    print(client_response)
                client.send(client_response.encode())
                time.sleep(0.1)
            print("closed")
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
