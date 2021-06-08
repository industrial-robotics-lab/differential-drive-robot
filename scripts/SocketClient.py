import socket

import serial
import os
import sys, select, termios, tty
from PIL import Image

ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

HOST = '192.168.0.110'  # IP address
# HOST = '127.0.0.1'  
PORT = 15000

vel = 60

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], None)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
 

if __name__=="__main__":
    print("current speed: ", vel)
    settings = termios.tcgetattr(sys.stdin)

    sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sck.connect((HOST, PORT))
    try:
        while True:
            key = getKey()
            print(key)
            if key:
                sck.sendall(str.encode(key))

            if(key == '\x03'): # ctrl+c
                break
            if(key == 'e'):
                vel += 5
                print("current speed: ", vel)
            if(key == 'q'):
                vel -= 5
                print("current speed: ", vel)


    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)                 








        