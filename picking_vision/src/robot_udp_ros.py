#!/usr/bin/env python3

from cmath import pi
import rospy
import socket
import time
import sys
from select import select
from moveit_msgs.msg import RobotTrajectory
import numpy as np

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

def getKey(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def udp_msg(msg_trj):

    print(len(msg_trj))
    # print(msg_trj*180/pi)

    # localIP     = "192.168.1.150"
    # # localIP     = "127.0.0.1"
    # localPort   = 7000
    # bufferSize  = 1024

    # # Create a datagram socket
    # UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    
    # # Bind to address and ip
    # UDPServerSocket.bind((localIP, localPort))

    # # clientAddressPort   = ("192.168.1.100", 7000)
    # clientAddressPort   = ("192.168.1.100", 700)

    # print("UDP server up and listening")

    # settings = saveTerminalSettings()

    # key_timeout = rospy.get_param("~key_timeout", 0.5)

    velocity = 10.0
    # Listen for incoming datagrams
    for i in range(len(msg_trj)):
        # print("wait connect....")
        msg_trj[i][0],msg_trj[i][1],msg_trj[i][2],msg_trj[i][3],msg_trj[i][4],msg_trj[i][5],velocity
        msgFromServer = "J,%0.0001f,%0.0001f,%0.0001f,%0.0001f,%0.0001f,%0.0001f,%0.1f\r" % (msg_trj[i][0]*180/pi,msg_trj[i][1]*180/pi,msg_trj[i][2]*180/pi,msg_trj[i][3]*180/pi,msg_trj[i][4]*180/pi,msg_trj[i][5]*180/pi,velocity)
        # print(msgFromServer)
        # msgFromServer1       = "J,-77.319,111.722,-35.426,0.972,-1.924,-0.463,10.0\r" # 2nd nxMarker pose
        bytesToSend  = str.encode(msgFromServer)

        # UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
        print(bytesToSend)
        time.sleep(1)

    # while not rospy.is_shutdown():

    #     key = getKey(settings, key_timeout)

    #     if key == "q":
    #         print("program end....")
    #         break

    #     if key == "s":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer       = "S\r"
    #         bytesToSend         = str.encode(msgFromServer)

    #         UDPServerSocket.sendto(bytesToSend, clientAddressPort)
    #         time.sleep(1)

    #         data, addressInfo = UDPServerSocket.recvfrom(100)
    #         # data = UDPClientSocket.recv(1024)
    #         print(data.decode('utf-8') + 'from' + str(addressInfo))
    #         time.sleep(1)

    #     if key == "c":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer       = "C\r"
    #         bytesToSend         = str.encode(msgFromServer)

    #         UDPServerSocket.sendto(bytesToSend, clientAddressPort)
    #         print(bytesToSend)
    #         time.sleep(1)

    #     if key == "o":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer       = "O\r"
    #         bytesToSend         = str.encode(msgFromServer)

    #         UDPServerSocket.sendto(bytesToSend, clientAddressPort)
    #         print(bytesToSend)
    #         time.sleep(1)

    #     if key == "m":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer       = "J,-93.640,108.780,-19.437,0.0,0.0,1.593,10.0\r"  # 1st marker
    #         bytesToSend         = str.encode(msgFromServer)

    #         UDPServerSocket.sendto(bytesToSend, clientAddressPort)
    #         print(bytesToSend)
    #         time.sleep(1) 

    #     if key == "n":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer1       = "J,-77.319,111.722,-35.426,0.972,-1.924,-0.463,10.0\r" # 2nd nxMarker pose
    #         bytesToSend1         = str.encode(msgFromServer1)

    #         UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
    #         print(bytesToSend1)
    #         time.sleep(1)

    #     if key == "x":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer1       = "J,-84.8986,112.216,-23.7825,20.9945,-38.8392,-27.9574,10.0\r" # 2nd initial pose
    #         # msgFromServer1       = "J,-85.1586,112.473,-24.1024,22.73,-38.1806,-29.344,10.0\r" # 2nd initial pose
    #         bytesToSend1         = str.encode(msgFromServer1)

    #         UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
    #         print(bytesToSend1)
    #         time.sleep(1)

    #     if key == "l":
    #         # Sending a msg to client
    #         print("wait connect....")
            
    #         msgFromServer1       = "J,-80.4037,73.4611,6.10624,20.5784,-28.9174,-9.26438,10.0\r" #Marker pose
    #         # msgFromServer1       = "J,-76.2191,76.6853,1.10051,20.1384,-26.9658,-83.6316,10.0\r" #Marker pose
    #         bytesToSend1         = str.encode(msgFromServer1)

    #         UDPServerSocket.sendto(bytesToSend1, clientAddressPort)
    #         print(bytesToSend1)
    #         time.sleep(1)

def trj_callback(trj_data):
    
    robot_trj = trj_data.joint_trajectory
    msg_trj = []

    points_cnt = len(robot_trj.points)
    for i in range(points_cnt):
        # print(robot_trj.points[i].positions)
        msg_trj.append(robot_trj.points[i].positions)
        
    udp_msg(msg_trj)



if __name__ == "__main__":
    rospy.init_node('robot_control', anonymous=True)
    print("UDP server up and listening")
    rospy.Subscriber("/trajectory", RobotTrajectory, trj_callback, queue_size=1)
    rospy.spin()
