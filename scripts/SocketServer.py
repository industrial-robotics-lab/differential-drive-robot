import socket, select
import serial
import sys
# import time, os
# import threading

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

# cap = cv2.VideoCapture(0)

# img_path = "CAM_ROBOT/rgb/"
# cam_txt_path = "CAM_ROBOT/rgb.txt"
# enc_txt_path = "CAM_ROBOT/enc.txt"
# os.system(str("rm -rf " + img_path))
# os.system(str("mkdir " + img_path))

# f_cam = open(cam_txt_path, "w")
# f_cam.writelines(["# gray images \n", "# file: cam from robot\n", "# timestamp filename \n"])
# f_enc = open(enc_txt_path, "w")



# def readDataFromSocket():


HOST = '192.168.0.110'
# HOST = '127.0.0.1'
PORT = 15000


# start = time.time()

# def readData():

sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sck.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sck.bind((HOST, PORT))
sck.listen()
conn, addr = sck.accept()
print("connect to: ", addr)

# sockets_to_check = [conn.fileno()]
while True:
    # ready_sockets = select.select(sockets_to_check, [], sockets_to_check, 0)
    # if (len(ready_sockets[0]) > 0):
    data = conn.recv(1)
    print(data)
    if (data == b'\x03'):
        # event_break.set() # for img read
        break
    
    s.write(data)
    # time.sleep(0.001)
conn.close()


# event_break = threading.Event()
# th_read = threading.Thread(target=readDataFromSocket)
# th_read.start()



# while True:
#     if(event_break.is_set()):
#         break

#     start = time.time()
   
#     timestamp = time.time()
#     timestamp_str = str(timestamp)

#     # ==== read and save data from encoders ===
#     # pos_msg = s.readline().decode('utf-8').rstrip()
#     # f_enc.writelines([timestamp_str, " ", pos_msg, " 0.000 0.000 0.000 0.000 0.000\n"])
#     # print(pos_msg, "\n")

#     # ==== read and save data from camera ====
    
#     ret, frame = cap.read()
#     img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     img_name = "{}.png".format(timestamp)
#     print("read img: ", img_name)

#     # cv2.imwrite("./" + img_path + img_name, img)
#     f_cam.writelines([timestamp_str, " ", "rgb/", str(timestamp), ".png", "\n"])

#     end = time.time()
#     delta = end-start

    

#     time.sleep(1/30) # 30 Hz

# th_read.join()

