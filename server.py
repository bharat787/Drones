import socket
import pickle

'''

msg = "Welcome to the server"
#--print(len(msg),10*' ',msg)
'''
d = ['hi', 'there']
class test_struct(Structure)
{
    _fields_ = [("x", c_float), 
                ("y", c_float),
                ("z", c_float),
                ("roll", c_float),
                ("yaw", c_float),
                ("pitch", c_float),
                ("message", c_String),
                ("status", c_Bolean)]
}
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), 1234)) 
s.listen(5)

while True:
    clientsocket, address = s.accept()
    print("Connection from",address,"has been established")

    
    msg = pickle.dumps(d)
    #print(msg)

    clientsocket.send((msg))
    clientsocket.close()
   