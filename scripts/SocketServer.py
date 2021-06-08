import socket, select
import serial
import sys


ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

s = serial.Serial()
s.baudrate = 9600
s.port = '/dev/ttyUSB0'   # for Raspberry
# s.port = '/dev/ttyACM0'     # for Linux
s.timeout = 0.5
s.open()

# ========================
# socket.SOCK_STREAM - TCP
# socket.SOCK_DGRAM - UDP
# AF_INET - IPv4
# ========================

HOST = '192.168.0.110'
# HOST = '127.0.0.1'
PORT = 15000


sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sck.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sck.bind((HOST, PORT))
sck.listen()
conn, addr = sck.accept()
print("connect to: ", addr)


while True:
    data = conn.recv(1)
    print(data)
    if (data == b'\x03'):
        break
    s.write(data)
conn.close()