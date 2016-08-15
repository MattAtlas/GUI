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
from multiprocessing import Process, Lock, Manager #multiprocessing


class Video(threading.Thread):
	def __init__(self, master):
		threading.Thread.__init__(self)

		self.master = master
		HOST = 'localhost'
		PORT = 8888
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		print "Socket created"

		server_address = (HOST,PORT)
		self.sock.connect(server_address)

		self.vidFrame = tk.Frame(master, width=640, height=480)
		self.vidFrame.pack(fill=tk.BOTH)
		print "Frame is packed. "

		self.vidLabel = tk.Label(self.vidFrame)
		self.vidLabel.pack(fill=tk.BOTH)
		#print "Label is packed.   About to enter showVideo()."
		self.showVideo()

	def recv_msg(self, buff):
		i = 0
		size = ""
		while i < 4:
			raw_size = self.sock.recv(1)
			size += "%s" % (raw_size.encode('hex'))
			i += 1
		size = int(size,16)
		print size
		data = ""
		while size > 0:
			data += self.sock.recv(buff)
			size -= buff
		print type(data)
		return data

	def showVideo(self):
		
		raw_data = self.recv_msg(1024)		# string
		b_data = np.fromstring(raw_data, dtype="uint8")
		decimg = cv2.imdecode(b_data,1)
		decimg = np.reshape(decimg,(480,640,3))
		img = Image.fromarray(decimg)
		imgtk = ImageTk.PhotoImage(image=img)
		self.vidLabel.configure(image=imgtk)
		self.vidLabel.image = imgtk
		self.master.update_idletasks()
		self.master.after(0, self.showVideo)

class tkinterGUI(tk.Frame):
	def __init__(self):
		root = tk.Frame.__init__(self)
		top=self.winfo_toplevel()
		top.wm_geometry("640x480")
		self.pack()

		print "just before calling videoThread."
		videoThread  = Video(self)

		videoThread.setDaemon(True)

		videoThread.start()

		#print '# active threads are ',threading.enumerate()


def startTkinter():
	root = tkinterGUI()
	root.master.title("video feed")
	print 'Entering Tkinter mainloop'
	root.mainloop()
	print 'Exited Tkinter mainloop'

def main():
	lock = Lock()
	manager = Manager()

	TkinterProcess = Process(name='Tkinter Process', target=startTkinter) #This process deals strictly with the GUI
	TkinterProcess.start()
	TkinterProcess.join()

if __name__ == '__main__':
	main()
