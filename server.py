import socket
import time
import cv2
import numpy as np
import sys
import struct

def send_msg(conn, msg):
	msg = struct.pack('>I',len(msg)) + msg
	conn.sendall(msg)


HOST = 'localhost'
PORT = 8888

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print "Socket created"

server_address = (HOST,PORT)

try:
	sock.bind(server_address)
	print "Socket bind complete"
except socket.error as msg:
	print "Bind failed. Error code : " + str(msg[0]) + " Message " + msg[1]
	sys.exit()

sock.listen(1)
print "Waiting for client to connect"

conn, addr = sock.accept()
print 'Connected with ' + addr[0] + ':' + str(addr[1])


#try:
#while True:

vc = cv2.VideoCapture(0)
#vc.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
#vc.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

retval,image = vc.read()

while retval:
	#print image.shape
	image = np.array(image,dtype="uint8")

	encode_param = [cv2.IMWRITE_JPEG_QUALITY,50]
	result,encimg = cv2.imencode(".jpg",image,encode_param)
	#decimg = cv2.imdecode(encimg,1)
	print encimg.shape
	print encimg.size
	print type(encimg)
	#print type(jimage)
	#cv2.imshow('Original image',jimage)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows() 
	#image = image.flatten().tostring()
	image = encimg.flatten().tostring()

	send_msg(conn, image)
	retval,image = vc.read()
	time.sleep(1/30)

#except KeyboardInterrupt:

print "closing socket"
sock.close()
