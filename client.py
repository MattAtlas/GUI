import socket
import time
import cv2
import numpy as np
import sys
import struct
import Image
import Tkinter as tk
from PIL import ImageTk, Image
import threading

class Video(threading.Thread):
	def __init__(self, root):
		threading.Thread.__init__(self)

		HOST = 'localhost'
		PORT = 8888
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		print "Socket created"

		server_address = (HOST,PORT)
		self.sock.connect(server_address)

		self.vidFrame = tk.Frame(root, width=640, height=480)
		#vidFrame.grid(row=0, column=0, padx=10, pady=2)
		self.vidFrame.pack(fill=tk.BOTH)

	def showVideo(self):
		raw_data = self.recv_msg(1024)		# string

		b_data = np.fromstring(raw_data, dtype="uint8")
		decimg = cv2.imdecode(b_data,1)
		decimg = np.reshape(decimg,(480,640,3))

		img = Image.fromarray(decimg)
		imgtk = ImageTk.PhotoImage(image=img)

		vidLabel = tk.Label(self.vidFrame, image=imgtk)
#		vidLabel.imgtk = imgtk
#		vidLabel.configure(image=imgtk)
		vidLabel.pack(fill=tk.BOTH)

	def recv_msg(self, buff):
		i = 0
		size = ""
		while i < 4:
			raw_size = self.sock.recv(1)
			size += "%s" % (raw_size.encode('hex'))
			i += 1
		size = int(size,16)
		data = ""
		while size > 0:
			data += self.sock.recv(buff)
			size -= buff
		return data

	def run(self, root):
		try:
			while True:	
				self.showVideo()
		except KeyboardInterrupt:
			self.sock.close()
			root.quit

class GUI(tk.Frame):
	def __init__(self):
		root = tk.Frame.__init__(self)

def startTkinter(): #This will start the GUI
    root = GUI()
    root.master.title("Azog") 


    print 'Entering Tkinter mainloop'
    root.mainloop()
    print 'Exited Tkinter mainloop'

root = tk.Tk()
root.geometry("640x480")
root.title("video")
videoClass = Video(root)
Video.run(root)
root.mainloop()
