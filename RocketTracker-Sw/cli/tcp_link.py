import socket


class TCPFakeSerial():
    def __init__(self, addr: str):
        aparts = addr.split(":")
        self.port = 2000
        self.addr = aparts[0]
        if len(aparts) > 1:
            self.port = int(aparts[1])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.addr, self.port))

    def write(self, b: bytes):
        self.sock.sendall(b)

    def read_all(self):
        return self.sock.recv(10)
