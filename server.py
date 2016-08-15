import socket
import time
import cv2
import numpy as np
import sys
import struct
import errno

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

vc = cv2.VideoCapture(0)


retval,image = vc.read()

while True:
	try:
		image = np.array(image,dtype="uint8")

		encode_param = [cv2.IMWRITE_JPEG_QUALITY,30]
		result,encimg = cv2.imencode(".jpg",image,encode_param)

		image = encimg.flatten().tostring()

		send_msg(conn, image)
		retval,image = vc.read()
		time.sleep(1/40)

	except socket.error, e:
		if isinstance(e.args, tuple):
			print "errno is %d" % e[0]
			if e[0] == errno.EPIPE:
			   # remote peer disconnected
			   print "Detected remote disconnect"
			else:
			   # determine and handle different error
			   pass
		else:
			print "socket error ", e
		conn.close()
		break
	except IOError, e:
		# Hmmm, Can IOError actually be raised by the socket module?
		print "Got IOError: ", e
		break

	print "closing socket"
