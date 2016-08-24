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
import Queue


class getFrame(threading.Thread):
	def __init__(self, master, q):
		threading.Thread.__init__(self)
		self.master = master
		self.q = q
		HOST = 'localhost'
		PORT = 8888
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		print "Socket created"

		ADDR = (HOST,PORT)
		self.sock.bind(ADDR)
		self.run()

	def recvAll(self):

		buff = 4096*2
		data = ""
		while True:
			raw_data = self.sock.recv(12+buff)
			F, N, I, i = struct.unpack('>IIHH', raw_data[0:12]) 		# actually [0:9]
			if i == 0:													# first packet of frame
				break
		data += raw_data[12:buff+12]									# actually [10:buff+10]
		frame = F
		n = 1
		while True:
			raw_data = self.sock.recv(12+buff)
			F, N, I, i = struct.unpack('>IIHH', raw_data[0:12]) 		# actually [0:9]
			if F != frame:
				data = None
				break
			data += raw_data[12:buff+12]								# actually [10:buff+10]
			n += 1
			if n > I:
				break
		return data


	def run(self):
		if self.q.empty() == True:
			data = self.recvAll()
			self.q.put(data)
			sys.stdout.write("Frames in queue : %s\r" % str(self.q.qsize()))
			sys.stdout.flush()
		self.master.update_idletasks()
		self.master.after(0, lambda: self.run())



class Video(threading.Thread):
	def __init__(self, master, q):
		threading.Thread.__init__(self)

		self.master = master
		self.q = q
		self.vidFrame = tk.Frame(master, width=640, height=480)
		self.vidFrame.pack(fill=tk.BOTH)
		self.vidLabel = tk.Label(self.vidFrame)
		self.vidLabel.pack(fill=tk.BOTH)

		self.showVideo()


	def showVideo(self):
		
		if self.q.empty() == False:
			raw_data = self.q.get()
			self.q.task_done()

			if raw_data != None:
				b_data = np.fromstring(raw_data, dtype="uint8")
				decimg = cv2.imdecode(b_data,1)
				decimg = np.reshape(decimg,(480,640,3))
				b,g,r = cv2.split(decimg)
				decimg = cv2.merge((r,g,b))

				img = Image.fromarray(decimg)
				imgtk = ImageTk.PhotoImage(image=img)
				self.vidLabel.configure(image=imgtk)
				self.vidLabel.image = imgtk

		self.master.update_idletasks()
		self.master.after(0, lambda: self.showVideo())




class tkinterGUI(tk.Frame):
	def __init__(self):
		root = tk.Frame.__init__(self)
		top = self.winfo_toplevel()
		top.wm_geometry("640x480")
		self.pack()

		q = Queue.Queue(maxsize=0)
		
		getFrameThread = getFrame(self, q)
		videoThread  = Video(self, q)

		getFrameThread.setDaemon(True)
		videoThread.setDaemon(True)
		
		videoThread.start()
		getFrameThread.start()
		print '# active threads are ',threading.enumerate()


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
