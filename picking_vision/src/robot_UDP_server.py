#!/usr/bin/env python3

import rospy
import socket
import time
import keyboard

localIP     = "192.168.1.150"
# localIP     = "127.0.0.1"
localPort   = 7000
bufferSize  = 1024

# msgFromServer       = "S\r"
# bytesToSend         = str.encode(msgFromServer)

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 
# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

# UDPServerSocket.listen()

# UDPClientSocket, addr = UDPServerSocket.accept()

# print('Connected by', addr)

# clientAddressPort   = ("192.168.1.100", 7000)
clientAddressPort   = ("192.168.1.100", 700)

print("UDP server up and listening")

# Listen for incoming datagrams
while(True):
    if keyboard.is_pressed("q"):
        print("program end....")
        break

    if keyboard.is_pressed("s"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer       = "S\r"
        bytesToSend         = str.encode(msgFromServer)

        UDPServerSocket.sendto(bytesToSend, clientAddressPort)
        time.sleep(1)

        data, addressInfo = UDPServerSocket.recvfrom(100)
        # data = UDPClientSocket.recv(1024)
        print(data.decode('utf-8') + 'from' + str(addressInfo))
        time.sleep(1)

    if keyboard.is_pressed("c"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer       = "C\r"
        bytesToSend         = str.encode(msgFromServer)

        UDPServerSocket.sendto(bytesToSend, clientAddressPort)
        print(bytesToSend)
        time.sleep(1)

    if keyboard.is_pressed("o"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer       = "O\r"
        bytesToSend         = str.encode(msgFromServer)

        UDPServerSocket.sendto(bytesToSend, clientAddressPort)
        print(bytesToSend)
        time.sleep(1)

    if keyboard.is_pressed("m"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer       = "J,-93.640,108.780,-19.437,0.0,0.0,1.593,10.0\r"  # 1st marker
        bytesToSend         = str.encode(msgFromServer)

        UDPServerSocket.sendto(bytesToSend, clientAddressPort)
        print(bytesToSend)
        time.sleep(1) 

    if keyboard.is_pressed("n"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer1       = "J,-77.319,111.722,-35.426,0.972,-1.924,-0.463,10.0\r" # 2nd nxMarker pose
        bytesToSend1         = str.encode(msgFromServer1)

        UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
        print(bytesToSend1)
        time.sleep(1)

    if keyboard.is_pressed("x"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer1       = "J,-84.8986,112.216,-23.7825,20.9945,-38.8392,-27.9574,10.0\r" # 2nd initial pose
        # msgFromServer1       = "J,-85.1586,112.473,-24.1024,22.73,-38.1806,-29.344,10.0\r" # 2nd initial pose
        bytesToSend1         = str.encode(msgFromServer1)

        UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
        print(bytesToSend1)
        time.sleep(1)

    if keyboard.is_pressed("l"):
        # Sending a msg to client
        print("wait connect....")
        
        msgFromServer1       = "J,-80.4037,73.4611,6.10624,20.5784,-28.9174,-9.26438,10.0\r" #Marker pose
        # msgFromServer1       = "J,-76.2191,76.6853,1.10051,20.1384,-26.9658,-83.6316,10.0\r" #Marker pose
        bytesToSend1         = str.encode(msgFromServer1)

        UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
        print(bytesToSend1)
        time.sleep(1)

        # msgFromServer2       = "J,0.001,125.727,-30,0.026,0.434,-0.012,10.0\r"
        # bytesToSend2         = str.encode(msgFromServer2)

        # UDPServerSocket.sendto(bytesToSend2, clientAddressPort)
        # print(bytesToSend2)

        # time.sleep(1)

    
    # bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
        
    # message = bytesAddressPair[0]
    # address = bytesAddressPair[1]
    
    # clientMsg = "Message from Client:{}".format(message)
    # clientIP  = "Client IP Address:{}".format(address)

    # print(clientMsg, clientIP)
        
    
    

    # bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    # print("recieve msg")
    
    # message = bytesAddressPair[0]
    # address = bytesAddressPair[1]
    
    # clientMsg = "Message from Client:{}".format(message)
    # clientIP  = "Client IP Address:{}".format(address)
    
    # print(clientMsg, clientIP)

    # time.sleep(5)
