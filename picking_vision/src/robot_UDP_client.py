#!/usr/bin/env python3

#import rospy
import socket

msgFromClient       = "Hello UDP Server"
bytesToSend         = str.encode(msgFromClient)

# serverAddressPort   = ("127.0.0.1", 20001)
serverAddressPort   = ("192.168.1.150", 7000)
bufferSize          = 1024

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket
UDPClientSocket.sendto(bytesToSend, serverAddressPort)

msgFromServer = UDPClientSocket.recvfrom(bufferSize)

msg = "Message from Server {}".format(msgFromServer[0])
print(msg)
