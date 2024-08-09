import socket

# print(socket.gethostname())
host = "192.168.4.1"
port = 2000                   # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
for i in range(20):
    s.sendall(b'Hello, world')
# # data = s.recv(1024)
s.close()
# # print('Received', repr(data))
