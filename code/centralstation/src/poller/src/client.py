import socket
import sys


class Client():

    DEFAULT_BUFFER_SIZE = 1024
    SERVER_PORT = 10019

    def __init__(self, port=SERVER_PORT):
        self.port = port

    def pollBasestation(self, address):
        return Client.poll(address, self.port)

    @staticmethod
    def poll(address, port):
        #Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        #Connect the socket to server
        #server_address = (SERVER_IP_ADDR, SERVER_PORT) #the address is defined as a pair (host,port)
        server_address = (address, port)
        sock.connect(server_address)

        try:
            #Client ready to read data from server
            
            # Look for the response
            data = sock.recv(Client.DEFAULT_BUFFER_SIZE)
            if(not data == 0):
                return data
            else:
                return ""

        finally:
            sock.close()
