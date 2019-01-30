from socket import *


def get_readings():
    with open('readings.json') as f:
        return f.read()


HOST = 'localhost'
PORT = 50000
BUFFER_SIZE = 1024
ADDRESS = (HOST, PORT)
server = socket(AF_INET, SOCK_STREAM)
server.bind(ADDRESS)
server.listen(2)

while True:
    print("waiting on connection")
    client, address = server.accept()
    print('connected from:', address)
    while 1:
        data = client.recv(1024).decode()
        if not data:
            break
        if data == "readings":
            data = get_readings()
        client.send(data.encode())
    client.close()

server.close()
