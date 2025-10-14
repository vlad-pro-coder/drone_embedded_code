import socket
s = socket.socket()
s.connect(("192.168.2.1", 5001))
print("Connected!")
