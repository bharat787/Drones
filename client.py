import socket
import pickle

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((socket.gethostname(), 1234))

full_msg = b''
while True:
    msg = s.recv(100)    #--wrote 8 instead of 1024 to simulate buffer management of large data
    if len(msg) <= 0:
        break          #--if stream of messages is over break the loop
    full_msg += msg #--keep on adding to the msg
    print(full_msg)
    d = pickle.loads(full_msg)

print(d)