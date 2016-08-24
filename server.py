import socket
import time
import cv2
import numpy as np
import sys
import struct
import errno
import threading
from Queue import Queue
#from matplotlib import pyplot as plt

def send_data(out_q, sock, addr):
	frame = 0
	while True:
		data = out_q.get()
		frame += 1
	
		buff = 4096*2
		
		N = len(data) 		# length of actual data in bytes
		n = N
		I = N/buff 			# number of required dgrams including 0
		i = 0
		for i in range(0,I+1):
			if n > buff:
				dgram = data[i*buff:((i+1)*buff)] 	
				header = struct.pack('>IIHH', frame, N, I, i)   # 10 bytes
				dgram = header + dgram
				sock.sendto(dgram, addr)
				n -= buff
			else:
				dgram = data[i*buff:N]
				header = struct.pack('>IIHH', frame, N, I, i)
				dgram = header + dgram
				sock.sendto(dgram, addr)
	
		
		#time.sleep(0.5)



def read_cam(in_q):
	start = time.time()
	time.sleep(0.01)
	while True:

		start = time.time()
		retval,image = vc.read()
		deltaT = time.time() - start
		#print "time to read camera: "+str(deltaT)+"\r"

		# Make into numpy array
		image = np.array(image,dtype="uint8")
		encode_param = [cv2.IMWRITE_JPEG_QUALITY,50]
		result,encimg = cv2.imencode(".jpg",image,encode_param)
		packet = encimg.flatten().tostring()
		in_q.put(packet)
		time.sleep(0.04)


q = Queue()


HOST = 'localhost'
PORT = 8888
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
addr = (HOST,PORT)

vc = cv2.VideoCapture(0)

t1 = threading.Thread(target=read_cam, args=(q,))
t2 = threading.Thread(target=send_data, args=(q,sock,addr))

t2.start()
t1.start()


