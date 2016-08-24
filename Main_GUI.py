#!/usr/bin/env python
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
# matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg #, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style, ticker
import struct
import csv
import traceback
import cv2
import sys
# import ADC #Import the ADC code so joystick values can be sent to the drone from the ground station

from PIL import ImageTk , Image # for image conversion
#import cv2 # OpenCV for video handling
import tkFont, threading, Queue, tkMessageBox
from time import strftime, sleep, time
from collections import deque, OrderedDict
import socket # for sending across UDP packets 
from multiprocessing.sharedctypes import Value, Array
from ctypes import Structure, c_double, c_short, c_long
from multiprocessing import Process, Lock, Manager #multiprocessing

from argparse import ArgumentParser

#from pymavlink import mavutil #Mavlink

allDronesList=['Othrod','The Great Goblin','Boldog','Ugluk','Bolg','Orcobal','More Orcs','Orc1','Orc2','Orc3','Orc4']
activeDronesList=['Othrod','Ugluk','Bolg','Orcobal']

style.use("ggplot") #Define what style to use for the Plot

class listener(threading.Thread):
	def __init__(self, sizeOfBuffer, messages, startLogging, Log_msgIDs, new_data, UDPmaster):
		threading.Thread.__init__(self)
		#Initialize the variables
		self.receivedPacketBuffer_1				= deque([], sizeOfBuffer)
		self.receivedPacketBuffer_2 			= deque([], sizeOfBuffer)
		self.receivedPacketBufferLock 			= threading.Lock()
		self.messages 							= messages
		self.sizeOfBuffer 						= sizeOfBuffer
		self.startLogging 						= startLogging		
		self.UDPmaster 							= UDPmaster		
		self.Log_msgIDs 						= Log_msgIDs
		
		#Initialize the variables for the Attitude Packet
		self.message_Attitude_Boot_Time 		= []
		self.message_Roll 						= []
		self.message_Pitch 						= []
		self.message_Yaw 						= []
		
		#Initialize the variables for the Battery Packet
		self.message_Battery_Boot_Time 			= []
		self.message_Battery_Voltage			= []
		
		#Initialize the variables for the Position Packet
		self.message_Position_Boot_Time 		= []
		self.message_X_Position 				= []
		self.message_Y_Position 				= []
		self.message_Z_Position 				= []
		
		self.Use_First_Buffer 					= True
		self.Use_Second_Buffer 					= False
		self.outfile 							= 'data.csv' #Initialize the File name all of the data will be logged into (May wish to make more than one file name or auto-generate one based on the start and end time of the data collection)
		self.new_data 							= new_data

	def run(self):
		i = 0
		with open(self.outfile, 'w') as csv_handle: #Define the csv handle to open the self.outfile and write to it
			while 1:
				i = i + 1
				sleep(.1)
				self.new_data.value = 0
				try:
					if self.receivedPacketBufferLock.acquire(1):
						data = self.UDPmaster.recv() #This should receive data from the socket
						
						if self.startLogging.value == 1:
						
							if self.Use_First_Buffer == True:
								self.receivedPacketBuffer_1.append(data)
							
							elif self.Use_Second_Buffer == True:
								self.receivedPacketBuffer_2.append(data)
							
						else:
							self.receivedPacketBuffer_1.append(data)
						
					else:
						print "Lock was not ON"
					
				finally:
					self.receivedPacketBufferLock.release()
					if i == 1:
						BootTime = time() #Determine the time when the program started
						
					if i%self.sizeOfBuffer == 0:
					
						for x in xrange(self.sizeOfBuffer):
							if self.startLogging.value == 1: #Check if the User chose to log data
								if self.Use_First_Buffer == True: #Use the first buffer if the second buffer is either being logged or on the first loop
									self.packet = self.UDPmaster.mav.parse_char(self.receivedPacketBuffer_1.popleft())
									
								elif self.Use_Second_Buffer == True: #Use the second bugger if the first buffer is being logged
									self.packet = self.UDPmaster.mav.parse_char(self.receivedPacketBuffer_2.popleft())
									
							else:
								self.packet = self.UDPmaster.mav.parse_char(self.receivedPacketBuffer_1.popleft())
							
							if self.packet is not None: #If data exists
								self.UDPmaster.post_message(self.packet)
								
								if self.packet.get_msgId() == 30: 							#Attitude Packet (Contains Roll, Pitch, Yaw)
									self.message_Attitude_Boot_Time.append(time() - BootTime)
									self.message_Roll.append(self.packet.roll)				#roll  : Roll angle (rad, -pi..+pi) (float)
									self.message_Pitch.append(self.packet.pitch)			#pitch : Pitch angle (rad, -pi..+pi) (float)
									self.message_Yaw.append(self.packet.yaw)				#yaw   : Yaw angle (rad, -pi..+pi) (float)
									
								elif self.packet.get_msgId() == 1: 							#Battery Packet
									self.message_Battery_Boot_Time.append(time() - BootTime)
									self.message_Battery_Voltage.append(self.packet.voltages)
									
								elif self.packet.get_msgId() == 104: 						#Position Packet
									self.message_Position_Boot_Time.append(time() - BootTime)
									self.message_X_Position.append(self.packet.x)
									self.message_Y_Position.append(self.packet.y)
									self.message_Z_Position.append(self.packet.z)
									
								if self.startLogging.value == True and self.packet.get_msgId() in self.Log_msgIDs[:]: # Check if the GUI should be Logging data
										csv_writer = csv.writer(csv_handle, delimiter =',') #Define the csv module, seperate the data with ',' and using the csv_handle
										UDPlogThread = logger(csv_writer, self.packet) #Start logging the data
										UDPlogThread.setDaemon(True)
										UDPlogThread.start()
										UDPlogThread.join()
								
								else:
									# Don't need to record data yet
									pass
							else:
								# No Packet!
								pass
								
							if x == self.sizeOfBuffer-1: #If the buffer is now full save all of the parsed data into the message dictionary for logging purposes
								self.messages['ATTITUDE'] = {'BootTime':self.message_Attitude_Boot_Time, 'Roll': self.message_Roll, 'Pitch': self.message_Pitch, 'Yaw': self.message_Yaw}
								self.messages['SYS_STATUS'] = {'BootTime':self.message_Battery_Boot_Time, 'Battery_Voltage': self.message_Battery_Voltage}
								self.messages['VICON_POSITION_ESTIMATE'] = {'BootTime': self.message_Position_Boot_Time, 'X': self.message_X_Position, 'Y': self.message_Y_Position, 'Z': self.message_Z_Position}
								#Boolean so the program knows that the buffer has been completely parsed and the data has been saved
								self.new_data.value = 1
								
								#Reset the variables which temporarily store the data
								self.message_Attitude_Boot_Time = []
								self.message_Roll = []
								self.message_Pitch = []
								self.message_Yaw = []
								
								self.message_Battery_Boot_Time = []
								self.message_Battery_Voltage = []
								
								self.message_Position_Boot_Time = []
								self.message_X_Position = []
								self.message_Y_Position = []
								self.message_Z_Position = []								
								
						if self.startLogging.value == 1: #Check if the User chose to log data
							if self.Use_First_Buffer == True: #If the first buffer was used this loop, use the second buffer on the next loop
								self.Use_First_Buffer = False
								self.Use_Second_Buffer = True
								
							elif self.Use_Second_Buffer == True: #If the second buffer was used this loop, use the first buffer on the next loop
								self.Use_First_Buffer = True
								self.Use_Second_Buffer = False
								
						else: #Use the first buffer only when no data is being logged
							self.Use_First_Buffer = True 
				
class logger(threading.Thread): #This class is called when certain packets are to be written in a text file
	def __init__(self,csv_writer, packet):
		threading.Thread.__init__(self)
		self.csv_writer = csv_writer
		self.packet = packet
		
	def run(self):
		try:
			self.csv_writer.writerow([self.packet]) #This creates a row with the current packet being written on it
			
		except(KeyboardInterrupt, SystemExit): #If a keyboard interrupt is detected then the logging will be exited
			raise
			
		except:
			traceback.print_exc()

class send(threading.Thread): #This class is called to send the drone heartbeat, joystick, and mode packets.
	def __init__(self, UDPmaster):
		threading.Thread.__init__(self)
		self.UDPmaster = UDPmaster
		
	def run(self):
		while True:
			self.UDPmaster.mav.heartbeat_send(1, 2, 3, 4, 5, 6) #Sending the drone heartbeat packets so it knows communication between the ground station and the drone is still active
			sleep(1) #Sleep for 1 second
		# self.UDPmaster.mav.mav_flight_ctrl_and_modes_send(chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, mav_flight_mode_ctrl, mav_flight_mode_auto, mav_flight_mode_kill)

#This class contains information pretaining to the drone currently being flown by this ground station
class myUAVThreadClass(threading.Thread): 
	def __init__(self,master):					# Need to bring in UAV info.. bat, name, signal
		threading.Thread.__init__(self)
		myUAVFrame = tk.Frame(master)
		myUAVFrame.place(x=0, y=0, width=0.5*w, height=h)
		myUAVFrame.rowconfigure(0, weight=1)
		myUAVFrame.rowconfigure(1, weight=1)
		myUAVFrame.rowconfigure(2, weight=1)
		myUAVFrame.rowconfigure(3, weight=1)
		myUAVFrame.columnconfigure(0, weight=1)
		
		UAVdataFrame = tk.Frame(master)
		UAVdataFrame.place(x=0.5*w, y=0, width=0.5*w, height=h)
		UAVdataFrame.rowconfigure(0, weight=1)
		UAVdataFrame.rowconfigure(1, weight=1)
		UAVdataFrame.rowconfigure(2, weight=1)
		UAVdataFrame.rowconfigure(3, weight=1)
		UAVdataFrame.columnconfigure(0, weight=1)	

		batteryLife = 75 #This variable contains the current [Voltage] of the battery
		signalStrength = 86 #This variable contains the signal strength of the connection between the ground station and the drone
		flightTime = 524.53 #This variable contains the total time the drone has been active for, starting from the time the BBB was turned on
		myUAV = 'Azog' #This variable contains the name of the drone which is currently connected
		
		batt = tk.StringVar()
		batt.set(batteryLife)
		sig  = tk.StringVar()
		sig.set(signalStrength)
		uptime = tk.StringVar()
		uptime.set(flightTime)
		uavName = tk.StringVar()
		uavName.set(myUAV)

		batteryLabel = tk.Label(myUAVFrame, text = " Battery Life: ", font="-weight bold")
		signalLabel  = tk.Label(myUAVFrame, text = " Signal Strength: ", font="-weight bold")
		upTimeLabel  = tk.Label(myUAVFrame, text = " Flight Time: ", font="-weight bold")
		myUAVName    = tk.Label(myUAVFrame, text = " UAV Name: ", font="-weight bold")

		batteryValue = tk.Label(UAVdataFrame, textvariable=batt, font="-weight bold")	
		signalValue = tk.Label(UAVdataFrame, textvariable=sig,  font="-weight bold")
		timeValue = tk.Label(UAVdataFrame, textvariable=uptime, font="-weight bold")	
		nameValue = tk.Label(UAVdataFrame, textvariable=uavName, font="-weight bold")		
				
		batteryLabel.grid(row=0, column=0, sticky=tk.E)
		signalLabel.grid(row=1, column=0, sticky=tk.E)
		upTimeLabel.grid(row=2, column=0, sticky=tk.E)
		myUAVName.grid(row=3, column=0, sticky=tk.E)

		batteryValue.grid(row=0, column=0, sticky=tk.W)
		signalValue.grid(row=1, column=0, sticky=tk.W)
		timeValue.grid(row=2, column=0, sticky=tk.W)
		nameValue.grid(row=3, column=0, sticky=tk.W)
		#batteryValue.grid(row=0, column=1, sticky=tk.W)
		
	def run(self):
		i=0
		# while i<6:
		# 	print 'i is =',i
		# 	# print '# active threads in MyUAV loop are',threading.enumerate()
		# 	sleep(5)
		# 	i+= 1	
			
class otherdrones(threading.Thread):
	def __init__(self,master):
		threading.Thread.__init__(self)
		otherDroneCanvas = tk.Canvas(master) # to add scroll bar
		otherDroneCanvas.place(x=w, y=0, width=screenW-w, height=h/3)
		# Intialize places for
		i=0 # counter for referencing objects in the list
		self.allDroneDict=dict() # initalizing empty dictionary 
		for orc in allDronesList:
			# otherDroneCanvas.columnconfigure(i,weight=1)
			self.allDroneDict[orc]=tk.Button(otherDroneCanvas,text=orc, bg = "gray14", fg="snow")
			self.allDroneDict[orc].pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
			i = i+1

	def run(self):
		i = 1

class loggingThreadClass(threading.Thread): #This class records any data which the user selects based on provided options, all the user has the option to stop and start the recording the data at any time
	
	def __init__(self, master, startBool, Log_msgIDs):
		threading.Thread.__init__(self)
		loggingFrame = tk.Frame(master)

		loggingFrame.place(x=w+vidW, y=0.5*screenH, width=w, height=0.5*screenH)
		
		log_iattitude = tk.IntVar()
		log_iposition = tk.IntVar()
		log_ivelocity = tk.IntVar()
		log_ibattery  = tk.IntVar()
		
		#Create the Logging Buttons
		log_attitudeCheckButton = tk.Checkbutton(loggingFrame, text = 'Attitude', font="-weight bold",\
		 variable = log_iattitude, command = lambda : self.logVariables('Attitude', log_iattitude.get()))

		log_positionCheckButton = tk.Checkbutton(loggingFrame, text = 'Position', font="-weight bold",\
		 variable = log_iposition, command = lambda : self.logVariables('Position', log_iposition.get()))
		
		log_velocityCheckButton = tk.Checkbutton(loggingFrame, text = 'Velocity', font="-weight bold",\
		 variable = log_ivelocity, command = lambda : self.logVariables('Velocity', log_ivelocity.get()))
		
		log_batteryCheckButton = tk.Checkbutton(loggingFrame, text =  'Battery', font="-weight bold",\
		 variable = log_ibattery, command = lambda : self.logVariables('Battery', log_ibattery.get()))		
		
		log_attitudeCheckButton.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
		log_positionCheckButton.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
		log_velocityCheckButton.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
		log_batteryCheckButton.pack( side = tk.TOP, fill = tk.BOTH, expand = 1)
		
		#log_startButton = tk.Button(log_startButtonFrame, text = 'Record', command = self.startRecording)
		#log_stopButton = tk.Button(log_stopButtonFrame, text = 'Stop', command = self.stopRecording)
		
		#log_startButton.pack(side = tk.BOTTOM, fill = tk.BOTH, expand = 1)
		#log_stopButton.pack(side = tk.BOTTOM, fill = tk.BOTH, expand = 1)
		
		self.Log_msgIDs = Log_msgIDs
		self.loggingVariables = []
		self.msgIDs = []
		self.startLogging = startBool
		
		#The message IDs for each packet
		Attitude_msgID = 30
		Position_msgID = 104
		Velocity_msgID = 2134124124
		Battery_msgID  = 1
		
	def Stop(self):
		self.quit()     # stops mainloop
		self.destroy()  # this is necessary on Windows to prevent
						# Fatal Python Error: PyEval_RestoreThread: NULL tstate

	def startRecording(self): #Start Recording (Logging) any data selected
		self.startLogging.value = 1
		
	def stopRecording(self): #Stop Recording (Logging) any data
		self.startLogging.value = 0
		
	def logVariables(self, var_Name, var_State, Attitude_msgID, Position_msgID, Velocity_msgID, Battery_msgID):# Based on the user choice only certain variables are recorded
	
		if var_State == 1: #If the variables was selected
			self.loggingVariables.append(var_Name) #Add the Name of the variable to the string list of variables being logged currently
			
			if 'Attitude' is var_Name: #If 'Attitude' was the option selected
				self.Log_msgIDs.append(Attitude_msgID) #Add the Attitude message ID to the message ID list being logged currently
				
			elif 'Position' is var_Name: #If 'Position' was the option selected
				self.Log_msgIDs.append(Position_msgID) #Add the Position message ID to the message ID list being logged currently
				
			elif 'Velocity' is var_Name: #If 'Velocity' was the option selected
				self.Log_msgIDs.append(Velocity_msgID) #Add the Velocity message ID to the message ID list being logged currently
				
			elif 'Battery' is var_Name: #If 'Battery' was the option selected
				self.Log_msgIDs.append(Battery_msgID) #Add the Battery message ID to the message ID list being logged currently
			
		elif var_State == 0: #If the variable was de-selected
			
			if 'Attitude' is var_Name: #If 'Attitude' was de-selected
				ivar_Name = self.Log_msgIDs.index(Attitude_msgID) #Find the index in the Log message ID list which contains 'Attitude'
				self.Log_msgIDs.pop(ivar_Name) #Remove 'Attitude' from the Log message ID list
				
			elif 'Position' is var_Name: #If 'Position' was de-selected
				ivar_Name = self.Log_msgIDs.index(Position_msgID) #Find the index in the Log message ID list which contains 'Position'
				self.Log_msgIDs.pop(ivar_Name) #Remove 'Position' from the Log message ID list
				
			elif 'Velocity' is var_Name: #If 'Velocity' was de-selected
				ivar_Name = self.Log_msgIDs.index(Velocity_msgID) #Find the index in the Log message ID list which contains 'Velocity'
				self.Log_msgIDs.pop(ivar_Name) #Remove 'Velocity' from the Log message ID list
				
			elif 'Battery' is var_Name: #If 'Battery' was de-selected
				ivar_Name = self.Log_msgIDs.index(Battery_msgID) #Find the index in the Log message ID list which contains 'Battery'
				self.Log_msgIDs.pop(ivar_Name) #Remove 'Battery' from the Log message ID list
			
			ivar_Name = self.loggingVariables.index(var_Name) 
			self.loggingVariables.pop(ivar_Name) #Remove the variable name that was de-selected from the logging variables string list
			
		self.Log_names = self.loggingVariables #Update the list of variables names being logged currently
		
class settingsThreadClass(threading.Thread): #User selected flight modes, only one modes is viable at any point in time from each of the following modes: Flight control mode, Flight auto mode, and Flight kill mode.
	def __init__(self,master):
		threading.Thread.__init__(self)
		settingsFrame = tk.Frame(master)
		#Place all of the buttons for each of the possible modes
		settingsFrame.place(x=w, y=int(h/3), width=screenW-w, height=int(2*h/3))

		# Mapping modes are : SLAM (0) or VICON pos input (1)
		# Flight modes are : Altitude (2) vs Manual Thrust (3) vs POS hold (4)
		# Pilot reference mode: global (5), First Person View (6), PPV (7)
		# Control mode: User (8) , Auto land (9), Come back home (10), Circle Mode (11)
		
		#Flight Control Modes are: Manual (0), Altitude (1), Attitude (2), Local Position (3), Global Position (4), Radial Position (5), Spherical Position (6), Follow Me (7)
		#Flight Auto Modes are: Manual (0), Emergency Land (1), Return Home (2), Wander (3)
		#Flight Kill Modes are: Fly (0), Kill Now (1)
		
		
		# mappingModeFrame = tk.Frame(settingsFrame)
		# flightModeFrame = tk.Frame(settingsFrame)
		# pilotReferenceModeFrame = tk.Frame(settingsFrame)
		# controlModeFrame = tk.Frame(settingsFrame)
		
		
		# FlightControlModeFrame.place()
		# FlightAutoModeFrame.place()
		# FlightKillModeFrame.place()
		
		# FlightKillModeFrame = tk.Frame(settingsFrame)
		FlightControlModeFrame = tk.Frame(settingsFrame)
		FlightAutoModeFrame = tk.Frame(settingsFrame)
		
		FlightControlModeFrame.place(x = 0, y = 0, width=screenW-w, height=h/3)
		FlightAutoModeFrame.place(x = 0, y = h/3, width=screenW-w, height=h/3)

		# m = tk.IntVar()
		# f = tk.IntVar()
		# p = tk.IntVar()
		# c = tk.IntVar()

		# default flight modes
		# m.set(0) #mappingMode = 0    
		# f.set(4) #flightMode = 4
		# p.set(5) #pilotReferenceMode=5
		# c.set(8) #controlMode= 8
		
		k = tk.IntVar()
		c = tk.IntVar()
		a = tk.IntVar()
		
		k.set(0) #KillMode = 0 (Fly)
		c.set(0) #ControlMode = 0 (Manual Control)
		a.set(0) #AutoMode = 0 (Manual Auto)
		
		# Emergency Kill Drone button
		killFrame = tk.Frame(master)
		killFrame.place(x=0, y=0.9*screenH, width=w, height=0.1*screenH)	
		killButton = tk.Button(killFrame, text="Kill Drone", font="-weight bold", command = lambda : FlightModeState(k,c,a), bg ="red")
		killButton.place(x=0, y=0, width=w, height=0.1*screenH)

		#Kill Mode Flight Buttons
		# KillModeButton_Fly 				= tk.Radiobutton(FlightKillModeFrame, text = "Fly", variable = k,
											# value = 0, indicatoron = 0, state = tk.ACTIVE, 
											# command = lambda : FlightModeState(k,c,a))
		# KillModeButton_Kill 			= tk.Radiobutton(FlightKillModeFrame, text = "Kill", variable = k,
											# value = 1, indicatoron = 0, state = tk.ACTIVE, 
											# command = lambda : FlightModeState(k,c,a))
							
		# KillModeButton_Fly.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)
		# KillModeButton_Kill.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)
		
		#Control Mode Flight Buttons
		ControlModeButton_Manual 		= tk.Radiobutton(FlightControlModeFrame, text = 'Manual', variable = c,
											value = 0, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Altitude 		= tk.Radiobutton(FlightControlModeFrame, text = 'Altitude', variable = c,
											value = 1, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Attitude 		= tk.Radiobutton(FlightControlModeFrame, text = 'Attitude', variable = c,
											value = 2, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Pos_Local 	= tk.Radiobutton(FlightControlModeFrame, text = 'Local Pos', variable = c,
											value = 3, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Pos_Global 	= tk.Radiobutton(FlightControlModeFrame, text = 'Global Pos', variable = c,
											value = 4, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Pos_Radial 	= tk.Radiobutton(FlightControlModeFrame, text = 'Radial Pos', variable = c,
											value = 5, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Pos_Spherical = tk.Radiobutton(FlightControlModeFrame, text = 'Spherical Pos', variable = c,
											value = 6, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		ControlModeButton_Pos_FollowMe 	= tk.Radiobutton(FlightControlModeFrame, text = 'Follow Me', variable = c,
											value = 7, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
										
		ControlModeButton_Manual.pack(		 side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Altitude.pack(	 side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Attitude.pack(	 side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Pos_Local.pack(	 side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Pos_Global.pack(	 side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Pos_Radial.pack(	 side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Pos_Spherical.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)
		ControlModeButton_Pos_FollowMe.pack( side = tk.LEFT, fill = tk.BOTH, expand = 1)
		
		#Auto Mode Flight Buttons
		AutoModeButton_Manual 			= tk.Radiobutton(FlightAutoModeFrame, text = 'Manual Auto', variable = a,
											value = 0, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		AutoModeButton_EmergencyLand 	= tk.Radiobutton(FlightAutoModeFrame, text = 'Emergency Land', variable = a,
											value = 1, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		AutoModeButton_ReturnHome 		= tk.Radiobutton(FlightAutoModeFrame, text = 'Return Home', variable = a,
											value = 2, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		AutoModeButton_Wander 			= tk.Radiobutton(FlightAutoModeFrame, text = 'Wander', variable = a,
											value = 3, indicatoron = 0, state = tk.ACTIVE,
											command = lambda : FlightModeState(k,c,a))
		
		AutoModeButton_Manual.pack(			side = tk.LEFT, fill = tk.BOTH, expand = 1)
		AutoModeButton_EmergencyLand.pack(	side = tk.LEFT, fill = tk.BOTH, expand = 1)
		AutoModeButton_ReturnHome.pack(		side = tk.LEFT, fill = tk.BOTH, expand = 1)
		AutoModeButton_Wander.pack(			side = tk.LEFT, fill = tk.BOTH, expand = 1)
		
		# mappingModeRadioButton0=tk.Radiobutton(mappingModeFrame, text="SLAM", variable=m, 
								# value=0,indicatoron=0,
								# state=tk.ACTIVE, command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# mappingModeRadioButton1=tk.Radiobutton(mappingModeFrame, text="VICON position input", 
								# variable=m,
								# value=1,indicatoron=0,
								# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# mappingModeRadioButton0.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# mappingModeRadioButton1.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)

		# flightModeRadioButton2=tk.Radiobutton(flightModeFrame, text="Altitude", variable=f, value=2,
								# indicatoron=0,
								# state=tk.ACTIVE, # set as default
								# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# flightModeRadioButton3=tk.Radiobutton(flightModeFrame, text="Manual Thrust", variable=f, value=3,
								# indicatoron=0,command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# flightModeRadioButton4=tk.Radiobutton(flightModeFrame, text="POS hold", variable=f, value=4,
								# indicatoron=0,
								# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# flightModeRadioButton2.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# flightModeRadioButton3.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# flightModeRadioButton4.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)

		# pilotReferenceModeRadioButton5=tk.Radiobutton(pilotReferenceModeFrame, text="Global", variable=p, value=5,
											# indicatoron=0,
											# state=tk.ACTIVE,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# pilotReferenceModeRadioButton6=tk.Radiobutton(pilotReferenceModeFrame, text="First Person View",
											# variable=p, value=6,indicatoron=0,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# pilotReferenceModeRadioButton7=tk.Radiobutton(pilotReferenceModeFrame, text="PPV", variable=p,
											# value=7,indicatoron=0,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# pilotReferenceModeRadioButton5.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# pilotReferenceModeRadioButton6.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# pilotReferenceModeRadioButton7.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)

		# controlModeRadioButton8=tk.Radiobutton(controlModeFrame, text="User", variable=c,
											# value=8,indicatoron=0,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# controlModeRadioButton9=tk.Radiobutton(controlModeFrame, text="Auto Land", variable=c,
											# value=9,indicatoron=0,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# controlModeRadioButton10=tk.Radiobutton(controlModeFrame, text="Return Home", variable=c,
											# value=10,indicatoron=0,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# controlModeRadioButton11=tk.Radiobutton(controlModeFrame, text="Hover", variable=c,
											# value=11,indicatoron=0,
											# command=lambda : sendSettingPacket(m.get(),f.get(),p.get(),c.get()))
		# controlModeRadioButton8.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# controlModeRadioButton9.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# controlModeRadioButton10.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		# controlModeRadioButton11.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
		
class statisticsThreadClass(threading.Thread): #Based on the User selected options this class will plot real time data from the drone
	def __init__(self, master, messages, new_data):
		threading.Thread.__init__(self)
		statisticsFrame = tk.Frame(master)		# this frame holds the plot and check boxes
		statisticsFrame.place(x=0, y=h, width=w, height=0.6*screenH)
		statisticsFrame.configure(bg='green')

		plotFrame 										= tk.Frame(statisticsFrame)
		plotFrameh 										= 0.5*screenH
		plotFrame.place(x=0, y=0, width=w, height=0.5*screenH)
		
		statButtonFrame 								= tk.Frame(statisticsFrame)
		statButtonFrame.place(x=0, y=0.5*screenH, width=w, height=0.1*screenH)

		
		self.stat_iXposition 							= tk.IntVar()
		self.stat_iYposition 							= tk.IntVar()
		self.stat_iZposition 							= tk.IntVar()
		self.stat_iroll 								= tk.IntVar()
		self.stat_ipitch 								= tk.IntVar()
		self.stat_iyaw 									= tk.IntVar()

		self.fig 										= plt.figure(figsize = (5,5), dpi = 75)
		self.ax 										= self.fig.add_subplot(111)

		self.canvas = FigureCanvasTkAgg(self.fig, plotFrame)
		self.canvas.get_tk_widget().grid(row = 0, column = 0, sticky = tk.N + tk.S + tk.W + tk.E)
		self.canvas.get_tk_widget().place(x=0, y=0, width=w, height=0.5*screenH)
		self.canvas.get_tk_widget().rowconfigure(0, weight = 1)
		self.canvas.get_tk_widget().columnconfigure(0, weight=1)		
		self.canvas.show()
		
		self.ax.set_ylabel('Position (m)')
		self.xPosition_line 							= self.ax.plot([], [])[0]
		self.yPosition_line 							= self.ax.plot([], [])[0]
		self.zPosition_line								= self.ax.plot([], [])[0]
		
		self.ax2 										= self.ax.twinx()
		self.ax2.set_ylabel('Rotation Angle (radians)')
		
		self.roll_line 									= self.ax2.plot([], [])[0]
		self.pitch_line 								= self.ax2.plot([], [])[0]
		self.yaw_line 									= self.ax2.plot([], [])[0]
		
		self.new_data 									= new_data
		self.messages 									= messages

		stat_xPositionCheckButton 	= tk.Checkbutton(statButtonFrame, text = 'X Pos', variable = self.stat_iXposition, command = lambda : self.Plot('X_Position', self.stat_iXposition.get(), self.canvas))
		stat_yPositionCheckButton 	= tk.Checkbutton(statButtonFrame, text = 'Y Pos', variable = self.stat_iYposition, command = lambda : self.Plot('Y_Position', self.stat_iYposition.get(), self.canvas))
		stat_zPositionCheckButton 	= tk.Checkbutton(statButtonFrame, text = 'Z Pos', variable = self.stat_iZposition, command = lambda : self.Plot('Z_Position', self.stat_iZposition.get(), self.canvas))
		stat_rollCheckButton 		= tk.Checkbutton(statButtonFrame, text = 'Roll',  variable = self.stat_iroll, command = lambda : self.Plot('Roll', self.stat_iroll.get(), self.canvas))
		stat_pitchCheckButton 		= tk.Checkbutton(statButtonFrame, text = 'Pitch', variable = self.stat_ipitch, command = lambda : self.Plot('Pitch', self.stat_ipitch.get(), self.canvas))
		stat_yawCheckButton 		= tk.Checkbutton(statButtonFrame, text = 'Yaw',   variable = self.stat_iyaw, command = lambda : self.Plot('Yaw', self.stat_iyaw.get(), self.canvas))

		stat_xPositionCheckButton.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)
		stat_yPositionCheckButton.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)
		stat_zPositionCheckButton.pack(side = tk.LEFT, fill = tk.BOTH, expand = 1)
		stat_rollCheckButton.pack( 	   side = tk.LEFT, fill = tk.BOTH, expand = 1)
		stat_pitchCheckButton.pack(    side = tk.LEFT, fill = tk.BOTH, expand = 1)
		stat_yawCheckButton.pack(      side = tk.LEFT, fill = tk.BOTH, expand = 1)

		self.Roll_Changed 								= False
		self.Pitch_Changed 								= False
		self.Yaw_Changed 								= False
				
		self.xPosition_line.set_data([],[])
		self.yPosition_line.set_data([],[])
		self.zPosition_line.set_data([],[])
		self.roll_line.set_data([],[])
		self.pitch_line.set_data([],[])
		self.yaw_line.set_data([],[])
		self.legend_changed = False
		
	def run(self):	#Run the Real Time Plot Function
		while 1:
			if self.new_data.value == 1:
				self.AnimatePlot(self.canvas,self.legend_changed)
			sleep(.12) #Sleep for .12 seconds			
		self.canvas.draw()
		
	def Plot(self, var_name, var_state, canvas): #If a option is de-selected by the user then the data will be removed from the Plot
		if var_state == 0:				
			if var_name is "X_Position":
				self.xPosition_line.set_data([],[])
				
			elif var_name is "Y_Position":
				self.yPosition_line.set_data([],[])
				
			elif var_name is "Z_Position":
				self.zPosition_line.set_data([],[])
				
			elif var_name is "Roll":
				print 'Stop Plotting Roll'
				self.roll_line.set_data([],[])
				
			elif var_name is "Pitch":
				print 'Stop Plotting Pitch'
				self.pitch_line.set_data([],[])
					
			elif var_name is "Yaw":
				print 'Stop Plotting Yaw'
				self.yaw_line.set_data([],[])		
			
	def AnimatePlot(self, canvas, legend_changed): #Real Time Plotter
		Prev_Rotation_Scaling_Factor = 0
		Rotation_Scaling_Factor = 0
		Prev_Position_Scaling_Factor = 0
		Position_Scaling_Factor = 0
		Scale = 2
		
		NumDataPlots = self.stat_iyaw.get() + self.stat_ipitch.get() + self.stat_iroll.get() + self.stat_iZposition.get() + self.stat_iYposition.get() + self.stat_iXposition.get()
		
		if NumDataPlots is 0: #If none of the options are selected for plotting
			Position_Lines, _ 							= self.ax.get_legend_handles_labels()
			Rotation_Lines, _ 							= self.ax2.get_legend_handles_labels()
			Position_Lables 							= None
			Rotation_Lables 							= None
			L 											= self.ax2.legend(Position_Lines+Rotation_Lines, 'None', bbox_to_anchor = (0., 1.02, 1., .102), loc=3,
																ncol= 1, mode="expand", borderaxespad=0.)
			L.remove()
			
		else: #If at least one option has been selected for plotting
			Position_Lines, Position_Lables 			= self.ax.get_legend_handles_labels() #Obtain the Position labels and handles
			Rotation_Lines, Rotation_Lables 			= self.ax2.get_legend_handles_labels() #Obtain the Rotation labels and handles
			
			if self.stat_iXposition.get() == 0: #If 'X' isn't currently selected
				if 'X' in Position_Lables: #If 'X' was previously selected then this is true
					Position_Lines.pop(Position_Lables.index('X')) #Remove 'X' from Position Handle
					Position_Lables.pop(Position_Lables.index('X')) #Remove 'X' from the Position Lable
					
			if self.stat_iYposition.get() == 0: #If 'Y' isn't currently selected
				if 'Y' in Position_Lables: #If 'Y' was previously selected then this is true
					Position_Lines.pop(Position_Lables.index('Y')) #Remove 'Y' from Position Handle
					Position_Lables.pop(Position_Lables.index('Y')) #Remove 'Y' from the Position Lable
					
			if self.stat_iZposition.get() == 0: #If 'Z' isn't currently selected
				if 'Z' in Position_Lables: #If 'Z' was previously selected then this is true
					Position_Lines.pop(Position_Lables.index('Z')) #Remove 'Z' from Position Handle
					Position_Lables.pop(Position_Lables.index('Z')) #Remove 'Z' from the Position Lable
					
			if self.stat_iroll.get() == 0: #If 'Roll' isn't currently selected
				if 'Roll' in Rotation_Lables: #If 'Roll' was previously selected then this is true
					Rotation_Lines.pop(Rotation_Lables.index("Roll")) #Remove 'Roll' from Position Handle
					Rotation_Lables.pop(Rotation_Lables.index("Roll")) #Remove 'Roll' from the Position Lable
					
			if self.stat_ipitch.get() == 0: #If 'Pitch' isn' currently selected
				if 'Pitch' in Rotation_Lables: #If 'Pitch' was previously selected then this is true
					Rotation_Lines.pop(Rotation_Lables.index("Pitch")) #Remove 'Pitch' from Position Handle
					Rotation_Lables.pop(Rotation_Lables.index("Pitch")) #Remove 'Pitch' from the Position Lable
					 
			if self.stat_iyaw.get() == 0: #If 'Yaw' isn' currently selected
				if 'Yaw' in Rotation_Lables: #If 'Yaw' was previously selected then this is true
					Rotation_Lines.pop(Rotation_Lables.index('Yaw')) #Remove 'Yaw' from Position Handle
					Rotation_Lables.pop(Rotation_Lables.index('Yaw')) #Remove 'Yaw' from the Position Lable
			
			if Rotation_Scaling_Factor == 0: #If the Scaling Factor for the Rotation Variables is Zero
				if ('(1/' + str(Rotation_Scaling_Factor) + ')*Roll') in Rotation_Lables:
					Rotation_Lines.pop(Rotation_Lables.index('(1/' + str(Rotation_Scaling_Factor) + ')*Roll'))
					Rotation_Lables.pop(Rotation_Lables.index('(1/' + str(Rotation_Scaling_Factor) + ')*Roll'))					
					
				elif ('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Roll') in Rotation_Lables:
					Rotation_Lines.pop(Rotation_Lables.index('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Roll'))
					Rotation_Lables.pop(Rotation_Lables.index('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Roll'))					
					
				if '(1/' + str(Rotation_Scaling_Factor) + ')*Pitch' in Rotation_Lables:
					Rotation_Lines.pop(Rotation_Lables.index('(1/' + str(Rotation_Scaling_Factor) + ')*Pitch'))
					Rotation_Lables.pop(Rotation_Lables.index('(1/' + str(Rotation_Scaling_Factor) + ')*Pitch'))					
					
				elif '(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Pitch' in Rotation_Lables:
					Rotation_Lines.pop(Rotation_Lables.index('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Pitch'))
					Rotation_Lables.pop(Rotation_Lables.index('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Pitch'))					
					
				if ('(1/' + str(Rotation_Scaling_Factor) + ')*Yaw') in Rotation_Lables:
					Rotation_Lines.pop(Rotation_Lables.index('(1/' + str(Rotation_Scaling_Factor) + ')*Yaw'))
					Rotation_Lables.pop(Rotation_Lables.index('(1/' + str(Rotation_Scaling_Factor) + ')*Yaw'))
					
				elif ('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Yaw') in Rotation_Lables:
					Rotation_Lines.pop(Rotation_Lables.index('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Yaw'))
					Rotation_Lables.pop(Rotation_Lables.index('(1/' + str(Prev_Rotation_Scaling_Factor) + ')*Yaw'))
					
			if Position_Scaling_Factor == 0:
				if ('(1/' + str(Position_Scaling_Factor) + ')*X') in Position_Lables:
					Position_Lines.pop(Position_Lables.index('(1/' + str(Position_Scaling_Factor) + ')*X'))
					Position_Lables.pop(Position_Lables.index('(1/' + str(Position_Scaling_Factor) + ')*X'))
				
				elif ('(1/' + str(Prev_Position_Scaling_Factor) + ')*X') in Position_Lables:
					Position_Lines.pop(Position_Lables.index('(1/' + str(Prev_Position_Scaling_Factor) + ')*X'))
					Position_Lables.pop(Position_Lables.index('(1/' + str(Prev_Position_Scaling_Factor) + ')*X'))
				
				if ('(1/' + str(Position_Scaling_Factor) + ')*Y') in Position_Lables:
					Position_Lines.pop(Position_Lables.index('(1/' + str(Position_Scaling_Factor) + ')*Y'))
					Position_Lables.pop(Position_Lables.index('(1/' + str(Position_Scaling_Factor) + ')*Y'))
				
				elif ('(1/' + str(Prev_Position_Scaling_Factor) + ')*Y') in Position_Lables:
					Position_Lines.pop(Position_Lables.index('(1/' + str(Prev_Position_Scaling_Factor) + ')*Y'))
					Position_Lables.pop(Position_Lables.index('(1/' + str(Prev_Position_Scaling_Factor) + ')*Y'))
				
				if ('(1/' + str(Position_Scaling_Factor) + ')*Z') in Position_Lables:
					Position_Lines.pop(Position_Lables.index('(1/' + str(Position_Scaling_Factor) + ')*Z'))
					Position_Lables.pop(Position_Lables.index('(1/' + str(Position_Scaling_Factor) + ')*Z'))
				
				elif ('(1/' + str(Prev_Position_Scaling_Factor) + ')*Z') in Position_Lables:
					Position_Lines.pop(Position_Lables.index('(1/' + str(Prev_Position_Scaling_Factor) + ')*Z'))
					Position_Lables.pop(Position_Lables.index('(1/' + str(Prev_Position_Scaling_Factor) + ')*Z'))
					
			L = self.ax2.legend(Position_Lines + Rotation_Lines, Position_Lables + Rotation_Lables, bbox_to_anchor = (0., 1.02, 1., .102), loc=3,
					ncol= NumDataPlots, mode="expand", borderaxespad=0.)
						
###########################################################################################################################################
######################################################## Position! ########################################################################
###########################################################################################################################################
		
		New_xPosition_y_data = self.messages['VICON_POSITION_ESTIMATE']['X']
		New_yPosition_y_data = self.messages['VICON_POSITION_ESTIMATE']['Y']
		New_zPosition_y_data = self.messages['VICON_POSITION_ESTIMATE']['Z']
		xPosition_y_data = np.append(self.xPosition_line.get_ydata(), New_xPosition_y_data)
		yPosition_y_data = np.append(self.yPosition_line.get_ydata(), New_yPosition_y_data)
		zPosition_y_data = np.append(self.zPosition_line.get_ydata(), New_zPosition_y_data)
		
		if (self.stat_iXposition.get() and self.stat_iYposition.get() and self.stat_iZposition.get()) == 1:
			yMax_X_Position = abs(max(New_xPosition_y_data))
			yMax_Y_Position = abs(max(New_yPosition_y_data))
			yMax_Z_Position = abs(max(New_zPosition_y_data))
			
			Max_Position = max(yMax_X_Position, yMax_Y_Position, yMax_Z_Position)
			Min_Max_Position = min(yMax_X_Position, yMax_Y_Position, yMax_Z_Position)
			Position_Scaling_Factor = int(Min_Max_Position/Max_Position)
			
			if Position_Scaling_Factor >= Scale:
				if yMax_X_Position > (yMax_Y_Position and yMax_Z_Position):
					Scale_X_Position = 1
					Scale_Y_Position = 0
					Scale_Z_Position = 0
					
				elif yMax_Y_Position > (yMax_X_Position and yMax_Z_Position):
					Scale_X_Position = 0
					Scale_Y_Position = 1
					Scale_Z_Position = 0
					
				elif yMax_Z_Position > (yMax_X_Position and yMax_Y_Position):
					Scale_X_Position = 0
					Scale_Y_Position = 0
					Scale_Z_Position = 1
			
			else:
				Scale_X_Position = 0
				Scale_Y_Position = 0
				Scale_Z_Position = 0
			
		elif (self.stat_iXposition.get() and self.stat_iYposition.get()) == 1:
			yMax_X_Position = abs(max(New_xPosition_y_data))
			yMax_Y_Position = abs(max(New_yPosition_y_data))
			
			Max_Position = max(yMax_X_Position, yMax_Y_Position)
			Min_Max_Position = min(yMax_X_Position, yMax_Y_Position)
			Position_Scaling_Factor = int(Min_Max_Position/Max_Position)
			
			if Position_Scaling_Factor >= Scale:
				if yMax_X_Position > yMax_Y_Position:
					Scale_X_Position = 1
					Scale_Y_Position = 0
					Scale_Z_Position = 0
					
				elif yMax_Y_Position > yMax_X_Position:
					Scale_X_Position = 0
					Scale_Y_Position = 1
					Scale_Z_Position = 0
					
			else:
				Scale_X_Position = 0
				Scale_Y_Position = 0
				Scale_Z_Position = 0
				
		elif (self.stat_iYposition.get() and self.stat_iZposition.get()) == 1:
			yMax_Y_Position = abs(max(New_yPosition_y_data))
			yMax_Z_Position = abs(max(New_zPosition_y_data))
			
			Max_Position = max(yMax_Y_Position, yMax_Z_Position)
			Min_Max_Position = min(yMax_Y_Position, yMax_Z_Position)
			Position_Scaling_Factor = int(Min_Max_Position/Max_Position)
			
			if Position_Scaling_Factor >= Scale:
				if yMax_Y_Position > yMax_Z_Position:
					Scale_X_Position = 0
					Scale_Y_Position = 1
					Scale_Z_Position = 0
					
				elif yMax_Z_Position > yMax_Y_Position:
					Scale_X_Position = 0
					Scale_Y_Position = 0
					Scale_Z_Position = 1
			
			else:
				Scale_X_Position = 0
				Scale_Y_Position = 0
				Scale_Z_Position = 0
					
		elif (self.stat_iXposition.get() and self.stat_iZposition.get()) == 1:
			yMax_X_Position = abs(max(New_xPosition_y_data))
			yMax_Z_Position = abs(max(New_zPosition_y_data))
			
			Max_Position = max(yMax_X_Position, yMax_Z_Position)
			Min_Max_Position = min(yMax_X_Position, yMax_Z_Position)
			Position_Scaling_Factor = int(Min_Max_Position/Max_Position)
			
			if Position_Scaling_Factor >= Scale:
				if yMax_X_Position > yMax_Z_Position:
					Scale_X_Position = 1
					Scale_Y_Position = 0
					Scale_Z_Position = 0
					
				elif yMax_Z_Position > yMax_X_Position:
					Scale_X_Position = 0
					Scale_Y_Position = 0
					Scale_Z_Position = 1
					
		else:
			Scale_X_Position = 0
			Scale_Y_Position = 0
			Scale_Z_Position = 0
					
		if self.stat_iXposition.get() == 1:
			X_Position_x = np.append(self.xPosition_line.get_xdata(), self.messages['VICON_POSITION_ESTIMATE']['BootTime'])			
		
			if Scale_X_Position == 1:
				X_Position_y =np.append(self.xPosition_line.get_ydata(), New_xPosition_y_data*(1/Position_Scaling_Factor))
				
			else:
				X_Position_y = xPosition_y_data
				
			if len(X_Position_x) == len(X_Position_y):	
				self.xPosition_line.set_xdata(X_Position_x)
				self.xPosition_line.set_ydata(X_Position_y)
				
			else:
				self.xPosition_line.set_xdata(X_Position_x[:len(X_Position_y)])
				self.xPosition_line.set_ydata(X_Position_y)
				
			if len(X_Position_y) > 0:
				BufferSize = len(self.messages['VICON_POSITION_ESTIMATE']['BootTime'])
				
				xMax_X_Position = self.messages['VICON_POSITION_ESTIMATE']['BootTime'][BufferSize-1]
				yMin_X_Position = min(X_Position_y)
				yMax_X_Position = max(X_Position_y)
				
				xMax_Position = max(xMax_X_Position)
				yMin_Position = min(yMin_X_Position)
				yMax_Position = max(yMax_X_Position)
			
			if Scale_X_Position == 1 and self.X_Position_Changed is False and '(1/' + str(Position_Scaling_Factor) + ')*X' not in Position_Lables:
				self.xPosition_line = self.ax.plot(self.xPosition_line.get_xdata(), self.xPosition_line.get_ydata(), 'b', label='(1/' + str(Position_Scaling_Factor) + 'X Position')[0]
				self.X_Position_Changed = True
				
			elif 'X Position' not in Position_Lables and Scale_X_Position == 0:
				self.xPosition_line = self.ax.plot(self.xPosition_line.get_xdata(), self.xPosition_line.get_ydata(), 'b', label='X Position')[0]
				self.X_Position_Changed = False
			
		if self.stat_iYposition.get() == 1:
			print 'Plot Y Position!'
			Y_Position_x = np.append(self.yPosition_line.get_xdata(), self.messages['VICON_POSITION_ESTIMATE']['BootTime'])
			
			if Scale_Y_Position == 1:
				Y_Position_y = np.append(self.yPosition_line.get_ydata(), New_yPosition_y_data*(1/Position_Scaling_Factor))
			
			else:
				Y_Position_y = yPosition_y_data
			
			if len(Y_Position_x) == len(Y_Position_y):
				self.yPosition_line.set_xdata(Y_Position_x)
				self.yPosition_line.set_ydata(Y_Position_y)
				
			else:
				self.yPosition_line.set_xdata(Y_Position_x[:len(Y_Position_y)])
				self.yPosition_line.set_ydata(Y_Position_y)
			
			if len(Y_Position_y) > 0:
				BufferSize = len(self.messages['VICON_POSITION_ESTIMATE']['BootTime'])
				
				xMax_Y_Position = self.messages['VICON_POSITION_ESTIMATE']['BootTime'][BufferSize-1]
				yMin_Y_Position = min(Y_Position_y)
				yMax_Y_Position = max(Y_Position_y)
			
				if self.stat_iXposition.get() == 1:
					xMax_Position = xMax_Y_Position
					yMin_Position = min(yMin_Position, yMin_Y_Position)
					yMax_Position = max(yMax_Position, yMax_Y_Position)
					
				else:
					xMax_Position = xMax_yPosition
					yMin_Position = yMin_yPosition
					yMax_Position = yMax_yPosition
					
			if Scale_Y_Position == 1 and self.Y_Position_Changed is False and '(1/' + str(Position_Scaling_Factor) + ')*Y Position' not in Position_Lables:
				self.yPosition_line = self.ax.plot(self.yPosition_line.get_ydata(), self.yPosition_line.get_ydata(), 'g',label='(1/' + str(Position_Scaling_Factor) + ')*Y Position')[0]
				self.Y_Position_Changed = True
				
			elif 'Y Position' not in Position_Lables and Scale_Y_Position == 0:
				self.yPosition_line = self.ax.plot(self.yPosition_line.get_ydata(), self.yPosition_line.get_ydata(), 'g',label='Y Position')[0]
				self.Y_Position_Changed = False
			
		if self.stat_iZposition.get() == 1:
			Z_Position_x = np.append(self.zPosition_line.get_xdata(), self.messages['VICON_POSITION_ESTIMATE']['BootTime'])			
			
			if Scale_Z_Position == 1:
				Z_Position_y = np.append(self.zPosition_line.get_ydata(), New_zPosition_y_data*(1/Position_Scaling_Factor))
				
			else:
				Z_Position_y = zPosition_y_data
			
			if len(Z_Position_x) == len(Z_Position_y):
				self.zPosition_line.set_xdata(Z_Position_x)
				self.zPosition_line.set_ydata(Z_Position_y)
			
			else:
				self.zPosition_line.set_xdata(Z_Position_x[:len(Z_Position_y)])
				self.zPosition_line.set_ydata(Z_Position_y)
			
			if len(Z_Position_y) > 0:
			
				BufferSize = len(self.messages['VICON_POSITION_ESTIMATE']['BootTime'])
				
				xMax_Z_Position = self.messages['VICON_POSITION_ESTIMATE']['BootTime'][BufferSize-1]
				yMin_Z_Position = min(Z_Position_y)
				yMax_Z_Position = max(Z_Position_y)
			
				if (self.stat_iXposition.get() or self.stat_iYposition.get()) == 1:
					xMax_Position = xMax_Z_Position
					yMin_Position = min(yMin_Position, yMin_Z_Position)
					yMax_Position = max(yMax_Position, yMax_Z_Position)
					
				else:
					xMax_Position = xMax_zPosition
					yMin_Position = yMin_zPosition
					yMax_Position = yMax_zPosition
			
			if Scale_Z_Position == 1 and self.Z_Position_Changed is False and '(1/' + str(Position_Scaling_Factor) + ')*Z Position' not in Position_Lables:
				self.zPosition_line = self.ax.plot(self.zPosition_line.get_xdata(), self.zPosition_line.get_ydata(), 'r',label= '(1/' + str(Position_Scaling_Factor) + ')*Z Position')[0]
				self.Z_Position_Changed = True
				
			elif 'Z Position' not in Position_Lables and Scale_Z_Position == 0:
				self.zPosition_line = self.ax.plot(self.zPosition_line.get_xdata(), self.zPosition_line.get_ydata(), 'r',label='Z Position')[0]
				self.Z_Position_Changed = False
				
###########################################################################################################################################
######################################################## Rotation! ########################################################################
###########################################################################################################################################

		New_Roll_y_data = self.messages['ATTITUDE']['Roll']
		New_Pitch_y_data = self.messages['ATTITUDE']['Pitch']
		New_Yaw_y_data = self.messages['ATTITUDE']['Yaw']
		Roll_y_data = np.append(self.roll_line.get_ydata(), New_Roll_y_data)
		Pitch_y_data = np.append(self.pitch_line.get_ydata(), New_Pitch_y_data)
		Yaw_y_data = np.append(self.yaw_line.get_ydata(), New_Yaw_y_data)

		
		
		if (self.stat_iroll.get() and self.stat_ipitch.get() and self.stat_iyaw.get()) == 1:
			yMax_Roll 	= abs(max(New_Roll_y_data))
			yMax_Pitch 	= abs(max(New_Pitch_y_data))
			yMax_Yaw 	= abs(max(New_Yaw_y_data))

			Max_Angle = max(yMax_Roll, yMax_Pitch, yMax_Yaw)
			Min_Max_Angle = min(yMax_Roll, yMax_Pitch, yMax_Yaw)
			Rotation_Scaling_Factor = int(Max_Angle/Min_Max_Angle)
			print 'Roll/Pitch/Yaw Scaling Factor: ' + str(Rotation_Scaling_Factor)
			if Rotation_Scaling_Factor >= Scale:
				if yMax_Roll > (yMax_Pitch and yMax_Yaw):
					Scale_Roll 	= 1
					Scale_Pitch = 0
					Scale_Yaw 	= 0
					
				elif yMax_Pitch > (yMax_Roll and yMax_Yaw):
					Scale_Roll = 0
					Scale_Pitch = 1
					Scale_Yaw = 0
					
				elif yMax_Yaw > (yMax_Roll and yMax_Pitch):
					Scale_Roll = 0
					Scale_Pitch = 0
					Scale_Yaw = 1
			
			else:
				Scale_Roll 	= 0
				Scale_Pitch = 0
				Scale_Yaw 	= 0
				
		elif (self.stat_iroll.get() and self.stat_ipitch.get()) == 1:
			yMax_Roll 	= abs(max(New_Roll_y_data))
			yMax_Pitch 	= abs(max(New_Pitch_y_data))
			
			Max_Angle = max(yMax_Roll, yMax_Pitch)
			Min_Max_Angle = min(yMax_Roll, yMax_Pitch)
			Rotation_Scaling_Factor = abs(int(Max_Angle/Min_Max_Angle))
			print 'Roll/Pitch Scaling Factor: ' + str(Rotation_Scaling_Factor)
			if Rotation_Scaling_Factor >= Scale:
				if yMax_Roll > yMax_Pitch:
					Scale_Roll = 1
					Scale_Pitch = 0
					Scale_Yaw 	= 0
					
				elif yMax_Pitch > yMax_Roll:
					Scale_Roll = 0
					Scale_Pitch = 1
					Scale_Yaw 	= 0
					
			else:
				Scale_Roll 	= 0
				Scale_Pitch = 0
				Scale_Yaw 	= 0
				
		elif (self.stat_ipitch.get() and self.stat_iyaw.get()) == 1:
			yMax_Pitch 	= abs(max(New_Pitch_y_data))
			yMax_Yaw 	= abs(max(New_Yaw_y_data))
			
			Max_Angle = max(yMax_Pitch, yMax_Yaw)
			Min_Max_Angle = min(yMax_Pitch, yMax_Yaw)
			Rotation_Scaling_Factor = int(Max_Angle/Min_Max_Angle)
			
			if Rotation_Scaling_Factor >= Scale:
				if yMax_Pitch > yMax_Yaw:
					Scale_Roll = 0
					Scale_Pitch = 1
					Scale_Yaw 	= 0
					
				elif yMax_Yaw > yMax_Pitch:
					Scale_Roll = 0
					Scale_Pitch = 0
					Scale_Yaw 	= 1
					
			else:
				Scale_Roll 	= 0
				Scale_Pitch = 0
				Scale_Yaw 	= 0
				
		elif (self.stat_iroll.get() and self.stat_iyaw.get()) == 1:
			yMax_Roll 	= abs(max(New_Roll_y_data))
			yMax_Yaw 	= abs(max(New_Yaw_y_data))
			
			Max_Angle = max(yMax_Roll, yMax_Yaw)
			Min_Max_Angle = min(yMax_Roll, yMax_Yaw)
			Rotation_Scaling_Factor = int(Max_Angle/Min_Max_Angle)
			print Rotation_Scaling_Factor
			if Rotation_Scaling_Factor >= Scale:
				if abs(yMax_Roll) > abs(yMax_Yaw):
					Scale_Roll = 1
					Scale_Pitch = 0
					Scale_Yaw = 0
					
				elif abs(yMax_Yaw) > abs(yMax_Roll):
					Scale_Roll = 0
					Scale_Pitch = 0
					Scale_Yaw = 1
			else:
				Scale_Roll 	= 0
				Scale_Pitch = 0
				Scale_Yaw 	= 0
					
		else:
			Scale_Roll 	= 0
			Scale_Pitch = 0
			Scale_Yaw 	= 0
		
		if self.stat_iroll.get() == 1:			
			Roll_x = np.append(self.roll_line.get_xdata(), self.messages['ATTITUDE']['BootTime'])
			
			if Scale_Roll == 1:
				Roll_y = np.append(self.roll_line.get_ydata(), New_Roll_y_data*(1/Rotation_Scaling_Factor))
				
			else:
				Roll_y = Roll_y_data
				
			if len(Roll_x) == len(Roll_y):
				self.roll_line.set_xdata(Roll_x)
				self.roll_line.set_ydata(Roll_y)
				
			else:
				self.roll_line.set_xdata(Roll_x[:len(Roll_y)])
				self.roll_line.set_ydata(Roll_y)
				
			if len(Roll_y) > 0:
				BufferSize = len(self.messages['ATTITUDE']['BootTime'])
				xMax_Roll = self.messages['ATTITUDE']['BootTime'][BufferSize-1]
				yMin_Roll = min(Roll_y)
				yMax_Roll = max(Roll_y)

				xMax_Angle = xMax_Roll
				yMin_Angle = yMin_Roll
				yMax_Angle = yMax_Roll
			
			if Scale_Roll == 1 and self.Roll_Changed is False and '(1/' + str(Rotation_Scaling_Factor) + ')*Roll' not in Rotation_Lables:
				self.roll_line 		= self.ax2.plot(self.roll_line.get_xdata(), self.roll_line.get_ydata(), 'c',label= '(1/' + str(Rotation_Scaling_Factor) + ')*Roll')[0]
				self.Roll_Changed = True
				
			elif 'Roll' not in Rotation_Lables and Scale_Roll == 0:
				self.roll_line 		= self.ax2.plot(self.roll_line.get_xdata(), self.roll_line.get_ydata(), 'c',label='Roll')[0]
				self.Roll_Changed = False
				
		if self.stat_ipitch.get() == 1:
			Pitch_x = np.append(self.pitch_line.get_xdata(), self.messages['ATTITUDE']['BootTime'])
			
			if Scale_Pitch == 1:
				Pitch_y = np.append(self.pitch_line.get_ydata(), New_Pitch_y_data*(1/Rotation_Scaling_Factor))
				
			else:
				Pitch_y = Pitch_y_data
			
			if len(Pitch_x) == len(Pitch_y):
				self.pitch_line.set_xdata(Pitch_x)
				self.pitch_line.set_ydata(Pitch_y)
				
			else:
				self.pitch_line.set_xdata(Pitch_x[:len(Pitch_y)])
				self.pitch_line.set_ydata(Pitch_y)
				
			if len(Pitch_y) > 0:
				BufferSize = len(self.messages['ATTITUDE']['BootTime'])
				
				xMax_Pitch = self.messages['ATTITUDE']['BootTime'][BufferSize-1]
				yMin_Pitch = min(Pitch_y)
				yMax_Pitch = max(Pitch_y)
			
				if self.stat_iroll.get() == 1:
					xMax_Angle = xMax_Pitch
					yMin_Angle = min(yMin_Angle, yMin_Pitch)
					yMax_Angle = max(yMax_Angle, yMax_Pitch)
					
				else:
					xMax_Angle = xMax_Pitch
					yMin_Angle = yMin_Pitch
					yMax_Angle = yMax_Pitch
			
			if Scale_Pitch == 1 and self.Pitch_Changed is False and '(1/' + str(Rotation_Scaling_Factor) + ')*Pitch' not in Rotation_Lables:
				self.pitch_line = self.ax2.plot(self.pitch_line.get_xdata(), self.pitch_line.get_ydata(), 'm',label= '(1/' + str(Rotation_Scaling_Factor) + ')*Pitch')[0]
				self.Pitch_Changed = True
			
			elif 'Pitch' not in Rotation_Lables and Scale_Pitch == 0:
				self.pitch_line = self.ax2.plot(self.pitch_line.get_xdata(), self.pitch_line.get_ydata(), 'm',label='Pitch' )[0]
				self.Pitch_Changed = False
			
		if self.stat_iyaw.get() == 1:
			print 'Plot Yaw data!'
			Yaw_x = np.append(self.yaw_line.get_xdata(), self.messages['ATTITUDE']['BootTime'])

			if Scale_Yaw == 1:
				Yaw_Scaled_data = [x*(1/float(Rotation_Scaling_Factor)) for x in New_Yaw_y_data]
				Yaw_y = np.append(self.yaw_line.get_ydata(), Yaw_Scaled_data)
				
			else:
				Yaw_y = Yaw_y_data
			
			if len(Yaw_x) == len(Yaw_y):
				self.yaw_line.set_xdata(Yaw_x)
				self.yaw_line.set_ydata(Yaw_y)
			else:
				self.yaw_line.set_xdata(Yaw_x[:len(Yaw_y)])
				self.yaw_line.set_ydata(Yaw_y)
								
			if len(Yaw_y) > 0:
				BufferSize = len(self.messages['ATTITUDE']['BootTime'])
				
				xMax_Yaw = self.messages['ATTITUDE']['BootTime'][BufferSize-1]
				yMin_Yaw = min(Yaw_y)
				yMax_Yaw = max(Yaw_y)
				
				if (self.stat_iroll.get() or self.stat_ipitch.get()) == 1:
					xMax_Angle = xMax_Yaw
					yMin_Angle = min(yMin_Angle, yMin_Yaw)
					yMax_Angle = max(yMax_Angle, yMax_Yaw)
					
				else:
					xMax_Angle = xMax_Yaw
					yMin_Angle = yMin_Yaw
					yMax_Angle = yMax_Yaw
				
			if Scale_Yaw == 1 and self.Yaw_Changed is False and '(1/' + str(Rotation_Scaling_Factor) + ')*Yaw' not in Rotation_Lables:
				self.yaw_line 		= self.ax2.plot(self.yaw_line.get_xdata(), self.yaw_line.get_ydata(), 'k',label= '(1/' + str(Rotation_Scaling_Factor) + ')*Yaw')[0]
				self.Yaw_Changed = True
				
			elif 'Yaw' not in Rotation_Lables and Scale_Yaw == 0:
				self.yaw_line 		= self.ax2.plot(self.yaw_line.get_xdata(), self.yaw_line.get_ydata(), 'k',label='Yaw')[0]
				self.Yaw_Changed = False

		Prev_Rotation_Scaling_Factor  = Rotation_Scaling_Factor
		
		if self.stat_iXposition.get() == 1 or self.stat_iYposition.get() == 1 or self.stat_iZposition.get() == 1:
			if xMax_Position-30 <= 0:
				self.ax.set_xlim([0,xMax_Position])
				
			else:
				self.ax.set_xlim([xMax_Position-30,xMax_Position])
				
			self.ax.set_ylim([yMin_Position - abs(yMin_Position/10), yMax_Position + abs(yMax_Position/10)])
			self.ax.get_yaxis().set_major_formatter(ticker.FuncFormatter(lambda y, p: format(float(y), ',')))
			
		if self.stat_iroll.get() == 1 or self.stat_ipitch.get() == 1 or self.stat_iyaw.get() == 1:
			self.ax2.set_ylim([yMin_Angle - abs(yMin_Angle/10) ,yMax_Angle + abs(yMax_Angle/10)])
			self.ax2.get_yaxis().set_major_formatter(ticker.FuncFormatter(lambda y, p: format(float(y), ',')))
			
			if xMax_Angle-30 <= 0:
				self.ax.set_xlim([0,xMax_Angle])
				
			else:
				self.ax.set_xlim([xMax_Angle-30,xMax_Angle])
			
		self.ax.get_xaxis().set_major_formatter(ticker.FuncFormatter(lambda x, p: format(int(x), ',')))			
		canvas.draw()



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
		self.master.after(20, lambda: self.run())



class Video(threading.Thread):
	# Manages video streaming and Video Controls
	def __init__(self, master, q):
		threading.Thread.__init__(self)
		self.q = q
		self.master = master
		self.vidFrame = tk.Frame(master, bg='orangered')
		self.vidFrame.place(x=w, y=h, width=vidW, height=vidH)
		#HOST = 'localhost'
		#PORT = 8888
		#self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		#self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		#print "Socket created"

		#server_address = (HOST,PORT)
		#self.sock.connect(server_address)	# TCP
		#self.sock.bind(server_address)		# UDP
		#print "bind successful"
		# Intialize vidControl Frame
		vidControl = tk.Frame(master,bg='cyan')
		vidControl.place(x=w+vidW, y=h, width=w, height=0.2*screenH)

		self.recordButton = tk.Button(vidControl, 
										text = "Record", 
										bd = 2,
										bg = "Red",
										command = self.recordVideo)

		self.recordButton.place(x=0, y=0, width=0.5*w, height=0.2*screenH)
		screenshotButton = tk.Button(vidControl, 
										text = "Screen Capture",
										command = self.screenshot)

		screenshotButton.place(x=0.5*w, y=0, width=0.5*w, height=0.2*screenH)

		self.vidLabel = tk.Label(self.vidFrame)
		self.vidLabel.pack()

		self.takeScreenShot = 0 # Intialize screenshot toggle to be zerod
		self.cameraChannelOnVideo = 0 # Intialize Camera Channel to default to zero
		self.recordingVideo = 0
		print "bottom of video init"
		self.showVideo()

	'''
	def recv_msg(self, buff):
		i = 0
		size = ""
		for i in range(0,4):
			raw_size = self.sock.recv(1)
			size +=  % (raw_size.encode('hex'))

		size = int(size,16)
		data = ""
		while size > 0:
			if size > buff:
				data += self.sock.recv(buff)
				size -= buff
			else:
				data += self.sock.recv(size)
				size = 0
		return data
	
	def recv_msg(self):
		buff = 4096*2
		data = ""
		while True:
			raw_data = self.sock.recv(10+buff)
			frame, N, I, i = struct.unpack('>HIHH', raw_data[0:10]) 	# actually [0:9]
			if i == 0:
				break
		data += raw_data[10:buff+10]								# actually [10:buff+10]
		n = 1
		while True:
			raw_data = self.sock.recv(10+buff)
			frame, N, I, i = struct.unpack('>HIHH', raw_data[0:10]) 	# actually [0:9]		
			data += raw_data[10:buff+10]								# actually [10:buff+10]
			n += 1
			if n > I:
				break
		return data
	'''
	def recordVideo(self):
		try:
			self.recordingVideo = 1
			fourcc = cv2.cv.CV_FOURCC('D','I','V','X')
			outputFPS = 20.0
			self.vidWriter = cv2.VideoWriter('Video_'+str(self.cameraChannelOnVideo)+
			               '_'+strftime("%c")+'.avi', fourcc, outputFPS, (640, 480), True)
			
			''' @jmaddocks - please add logging code here'''

			self.recordButton.configure(text="Stop", 
										bg= "black",
										fg ="snow",
										command=self.stopVideoRecord)
		except:
			print "Something bad happened with recordVideo :("

	def stopVideoRecord(self):
		self.vidWriter.release()
		self.recordingVideo = 0
		print "Stopped Recording Video"
		self.recordButton.configure(text="Record",
									bg="Red",
									fg ="Black",
									command= self.recordVideo)
		#self.vidWriter.release()

	def screenshot(self):
		self.takeScreenShot = 1
		print 'Took a Screenshot!'
		
	def showVideo(self):

		if self.q.empty() == False:
			raw_data = self.q.get()
			self.q.task_done()

			if raw_data != None:
				b_data = np.fromstring(raw_data, dtype="uint8")
				decimg = cv2.imdecode(b_data,1)
				decimg = np.reshape(decimg,(480,640,3))


				if self.recordingVideo == 1:
					self.vidWriter.write(decimg)

				b,g,r = cv2.split(decimg)
				decimg = cv2.merge((r,g,b))
				img = Image.fromarray(decimg)

				# save image if screenshot toggle is on
				if self.takeScreenShot == 1:
					img.save('Camera('+str(self.cameraChannelOnVideo)+')_'+strftime("%c"),'jpeg')
					self.takeScreenShot=0

				if screenW == 1920:
					img = img.resize((960,720),Image.ANTIALIAS) ##### sloppy... fix this

				imgtk = ImageTk.PhotoImage(image=img)
				self.vidLabel.configure(image=imgtk)
				self.vidLabel.image = imgtk

		self.master.update_idletasks()
		self.master.after(0, lambda: self.showVideo())

		
class tkinterGUI(tk.Frame):
	def __init__(self, messages, startBool, Log_msgIDs, new_data):
		root = tk.Frame.__init__(self)
		#self.attributes('-zoomed', True)

		# make top level of the application stretchable and space filling
		top = self.winfo_toplevel()
		q = Queue.Queue()
		global screenH, screenW, vidH, vidW, h, w       
		screenH = top.winfo_screenheight() - 50 # take 25 pixels to account for top bar x 2
		screenW = top.winfo_screenwidth()
		print screenH, screenW
		# Defining gemotery of outermost Frame 
		geom_string = "%dx%d+0+0" % (screenW,screenH)
		# Assigning max height and width to outer Frame - Maximize Frame Size
		top.wm_geometry(geom_string)
		#top.attributes('-zoomed', True)
		self.place(x=0, y=0, width=screenW, height=screenH)
		# Retrive scalled dimensions according to schema 
		[vidH, vidW, h, w] = self.masterWidgetSizes()

		getFrameThread = getFrame(self,q)
		videoThread = Video(self,q)
		settingsThread = settingsThreadClass(self)
		loggingThread = loggingThreadClass(self, startBool, Log_msgIDs)
		statisticsThread = statisticsThreadClass(self, messages, new_data)
		myUAVThread = myUAVThreadClass(self)
		otherDrones = otherdrones(self)

		getFrameThread.setDaemon(True)
		videoThread.setDaemon(True)
		settingsThread.setDaemon(True)
		loggingThread.setDaemon(True)
		statisticsThread.setDaemon(True)
		myUAVThread.setDaemon(True)
		otherDrones.setDaemon(True)

		getFrameThread.start()
		videoThread.start() # becomes mainthread
		settingsThread.start()
		loggingThread.start()
		statisticsThread.start()
		myUAVThread.start()
		otherDrones.start()

		print '# active threads are ',threading.enumerate()
		# top.resizable(0,0)
		
	def masterWidgetSizes(self):

		'''
		Get Video Dimensions from Primary Camera
		CAUTION : If primary and Secondary Cameras have different 
		dimensions then scaling happens with respect to primary camera
		'''  


		# change this later based on UDP stream
		vidH = int(screenH*(0.7))
		vidW = int(screenW*(0.5))
		h 	 = int(screenH - vidH)
		w 	 = int((screenW - vidW)*0.5)
		print vidH, vidW, h, w
		'''
		#vidW=800
		#vidH=600
		print vidW, vidH
		camAspectRatio = vidW/vidH

		if screenH - vidH < 100: # Not enough height for other widgets
			# Reduce video height
			vidH=screenH-100
			vidW=int(vidH*camAspectRatio)
			h= 100
			w= int(0.5*(screenW-vidW))

		if screenW - vidW < 200:# Not enough width for other widgets
			vidW=screenW-200
			vidH=int(vidW/camAspectRatio)
			h= screenH-vidH
			w= 100

		else: # Enough width and Height for all widgets
			h = (screenH-vidH)
			w = int(0.5*(screenW-vidW))
		'''
		return vidH, vidW, h, w

		'''
		h= (screenH-vidH)
		w= 0.5*(screenW-vidW)
		 ___________________________________________________________
		|               |                                           |
		|   myDrone     |_________________otherDrones_______________|        
		|     wL, h     |_________________xyzpry____________________|
		|_______________|___________________________________________|
		|               |                               | vidControl|
		|    Status     |            Video              |   wR,     |
		|     wL,       |       Native Camera           |  1/4*vidH |
		|   3/4*vidH    |          Resolution           |___________|
		|               |         vidW*vidH             |           |
		|               |                               | Logging   |
		|               |                               | wR,       |
		|_______________|                               | 3/4*vidH  |
		|    kill!      |                               |           |
		|      wL,      |                               |           |
		|  1/4*vidH     |                               |           |
		|_______________|_______________________________|___________|
		'''
'''		
def udpConnection():
	ClientIPaddress = '192.168.7.2' # IP to send the packets
	portNum = 14551 # port number of destination

	device = 'udpout:' + str(ClientIPaddress) + ':' + str(portNum)
	baudrate = 57600
	
	parser = ArgumentParser(description=__doc__)
	parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
	parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
					  default=255, help='MAVLink source system for this GCS')
	parser.add_argument("--showmessages", action='store_true',
					  help="show incoming messages", default=False)
	args = parser.parse_args()
	
	master = mavutil.mavlink_connection(device, baud=baudrate) # create a mavlink serial instance
	
	return master
	'''
	
def UDP_Communication(messages, startLogging, Log_msgIDs, new_data):
	sizeOfBuffer = 16
	UDPmaster = udpConnection()
	
	UDPSendingThread = send(UDPmaster)
	UDPlistenThread = listener(sizeOfBuffer, messages, startLogging, 
								Log_msgIDs, new_data, UDPmaster) # sizeOfRingBuffer
	
	UDPlistenThread.setDaemon(True)
	UDPSendingThread.setDaemon(True)
	
	print('UDP Communication process starting')
	
	UDPSendingThread.start()
	UDPlistenThread.start()
	
	UDPSendingThread.join()
	UDPlistenThread.join()

	# Declaring global for killUDPprocesscounter
	"""
	print "UDP process started"

	i=0
	while(killUDPprocessCounter):
		i=i+1
		sleep(0.05)
		if i%20==0:
			print "UDP counter is",killUDPprocessCounter, "i is ", i

	print "Came out of while loop"
	"""

def closeProgram():
	global killUDPprocessCounter
	killUDPprocessCounter=0
	print "here I am ", killUDPprocessCounter
	
def startTkinter(PlotPacket,startBool, msgIDs, new_data): #This will start the GUI
	root = tkinterGUI(PlotPacket,startBool, msgIDs, new_data)
	root.master.title("Azog") # Name of current drone, Here it is Azog
	# root.master.attributes('-zoomed', True)
	# root.master.attributes('-fullscreen', True)
	print 'Entering Tkinter mainloop'
	root.mainloop()
	print 'Exited Tkinter mainloop'

def FlightModeState(k,c,a):
	print "New Settings received :",'Kill Mode',k,'\tControl Mode :',c,'\tAuto Mode',p
	
def sendSettingPacket(m,f,p,c):
	# m - Mapping modes are : SLAM (0) or VICON pos input (1)
	# f - Flight modes are : Altitude (2) vs Manual Thrust (3) vs POS hold (4)
	# p - Pilot reference mode: global (5), First Person View (6), PPV (7)
	# c - Control mode: User (8) , Auto land (9), Come back home (10), Circle Mode (11)
	print "New Settings received :",'Mapping Mode',m,'\tFlight Mode :',f,'\tPilot Reference Mode',p,'\tControl Mode',c
	# mav_flight_mode_ctrl        : (See MAV_CTRL_MODE) Valid options are: MAV_CTRL_MODE_MANUAL = 0, MAV_CTRL_MODE_ALTITUDE = 1, MAV_CTRL_MODE_ATTITUDE = 2, MAV_CTRL_MODE_POS_LOCAL = 3, MAV_CTRL_MODE_POS_GLOBAL = 4, MAV_CTRL_MODE_POS_RADIAL = 5, MAV_CTRL_MODE_POS_SPHERICAL = 6, MAV_CTRL_MODE_POS_FOLLOW_ME = 7 (uint8_t)
	# mav_flight_mode_auto        : (See MAV_AUTO_MODE) Valid options are: MAV_AUTO_MODE_MANUAL = 0, MAV_AUTO_MODE_EMERGENCY_LAND = 1, MAV_AUTO_MODE_RETURN_TO_HOME = 2, MAV_AUTO_MODE_WANDER = 3 (uint8_t)
	# mav_flight_mode_kill        : (See MAV_KILL) Valid options are: MAV_KILL_SWITCH_OFF = 0, MAV_KILL_NOW = 1 (uint8_t)
	#mav_flight_ctrl_and_modes_send(chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, mav_flight_mode_ctrl, mav_flight_mode_auto, mav_flight_mode_kill)
	#mav_flight_ctrl_and_modes_send(chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, mav_flight_mode_ctrl, mav_flight_mode_auto, mav_flight_mode_kill)
	
def killDroneMethod():
	print 'this should send a specific MAVlink packet'

class AutoScrollbar(tk.Scrollbar): #This class contains all of the drones which part of the swam
	# a scrollbar that hides itself if it's not needed.  only
	# works if you use the grid geometry manager.
	def set(self, lo, hi):
		if float(lo) <= 0.0 and float(hi) >= 1.0:
			# grid_remove is currently missing from Tkinter!
			self.tk.call("grid", "remove", self)
		else:
			self.grid()
		tk.Scrollbar.set(self, lo, hi)
	def pack(self, **kw):
		raise TclError, "cannot use pack with this widget"
	def place(self, **kw):
		raise TclError, "cannot use place with this widget"

def main():
	#global udpProcess # try to kill updprocess using startTkinter
	lock = Lock()
	manager = Manager()
	startLogging = Value('i', 0, lock = lock) 	# shared memory maps ( integer, init 0, lock )
	new_data = Value('i', 0, lock = lock)
	messages = manager.dict()
	Log_msgIDs = manager.list()

	# udpSendingProcess = Process(name = 'UDP Sending Process', target = UDP_Sending, args = ())
	udpCommunicationProcess = Process(name = 'UDP Communication Process', target = UDP_Communication, args=(messages, startLogging, Log_msgIDs, new_data)) #This process contains the sending and receiving threads
	TkinterProcess = Process(name='Tkinter Process', target=startTkinter, args=(messages, startLogging, Log_msgIDs, new_data)) #This process deals strictly with the GUI
	
	# udpSendingProcess.start()
	udpCommunicationProcess.start()
	TkinterProcess.start()
	
	# udpSendingProcess.join()
	udpCommunicationProcess.join()
	TkinterProcess.join()
	
if __name__ == '__main__':
	main()
