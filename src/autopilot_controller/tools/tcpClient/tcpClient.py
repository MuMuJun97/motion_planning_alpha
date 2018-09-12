# -*- coding: utf-8 -*-
import socket
import sys

if len(sys.argv) == 1:
    noArgv = True
    msgIndex = 0
else:
    noArgv = False
    msgIndex = int(sys.argv[1])

msgs = ['start 3*10_HW1_SW 1 1',  # 0
        'stop 3*10_HW1_SW 1 1',  # 1
        'restart 3*10_HW1_SW 1 1',  # 2
        'reset ControllerWatchdog',  # 3

        'start VotingActuatorsServer',  # 4
        'stop VotingActuatorsServer'  # 5
        ]

serverIP = '127.0.0.1'  # localhost
# serverIP = '192.168.70.130'    # controllerpi1
# serverIP = '192.168.70.10'     # actuatorpi

serverPort = 30006  # votingActuator
# serverPort = 30007   # controllerWatchdog
data = msgs[msgIndex]

if noArgv:
    serverIP = raw_input("Please input serverIP:\n")
    serverPort = input("Please input serverPort:\n")
    data = raw_input("Please input message data:\n")

print("send \"%s\" to %s:%d" % (data, serverIP, serverPort))

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((serverIP, serverPort))

sock.send(data.encode())

sock.close()
