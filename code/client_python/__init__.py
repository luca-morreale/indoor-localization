import socket
import sys

DEFAULT_BUFFER_SIZE = 1024
SERVER_IP_ADDR = '192.168.1.19'
SERVER_PORT = 10019

#Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#Connect the socket to server
#server_address = (SERVER_IP_ADDR, SERVER_PORT) #the address is defined as a pair (host,port)
server_address = ('localhost', 10000)
print('Connecting to %s port %s' %server_address)
sock.connect(server_address)

try:
    #Client ready to read data from server
    print ('Start reading from server')

    # Look for the response
    data = sock.recv(DEFAULT_BUFFER_SIZE)
    if(not data == 0):
        print ("Received data: %s, (%d bytes)" %(data, len(data)))

    else:
        print("Connection is broken")

finally:
    print ("Closing socket")
    sock.close()