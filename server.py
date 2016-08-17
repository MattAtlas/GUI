import socket
import time
import cv2
import numpy as np
import sys
import struct
import errno

# Function to back the length of message with message and send to socket
def send_msg(conn, msg):
	msg = struct.pack('>I',len(msg)) + msg
	#print len(msg)
	conn.sendall(msg)


# Get TCP socket ready for client.  Code continues after connection
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

# OpenCV camera object
vc = cv2.VideoCapture(1)

while True:
	try:
		retval,image = vc.read()
		# Rearrang the color channels
		b,g,r = cv2.split(image)
		image = cv2.merge((r,g,b))
		# Make into numpy array
		image = np.array(image,dtype="uint8")
		# Encode into jpeg 
		encode_param = [cv2.IMWRITE_JPEG_QUALITY,80]
		result,encimg = cv2.imencode(".jpg",image,encode_param)
		# Flatten and make into string ready for transmission
		packet = encimg.flatten().tostring()
		# Send packet to socket
		send_msg(conn, packet)
		# Wait time for next frame
		time.sleep(0.04)

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

