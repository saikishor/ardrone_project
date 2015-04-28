#!/usr/bin/env python

# https://github.com/saikishor

# It can command takeoff/landing/emergency as well as drone movement as well as for command processing
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_project')
import rospy
import time

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from sensor_msgs.msg import Image    	 # for receiving the video feed

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

from PySide import QtCore, QtGui

#constant value that is used
COMMAND_PERIOD = 100 #ms
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms

class DroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		self.altd = -1
		rospy.logwarn('DroneController')
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 

		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand) #this is for safety of the drone

	def ReceiveImage(self,data):
			self.image = data # Save the ros image for processing by the display thread

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		self.altd = navdata.altd
		self.rotZ = navdata.rotZ

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
#			self.pubTakeoff.publish(Empty())
			rospy.logwarn('Take-off Done Successfully')
		rospy.logwarn('Takeoff command sent')

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
#		self.pubLand.publish(Empty())
		rospy.logwarn('land command sent')
		rospy.logwarn('landed Successfully DroneController')

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)

class VideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# Construct the parent class
		super(VideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)

		# Subscribe to the /ardrone/navdata/altd topic, of message type 1, and call self.altitude when a message is received
		#self.subNavdata = rospy.Subscriber('/ardrone/navdata/altd',1,self.altitude) 
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%) (Altitude : {}mm) (rotZ : {}deg)'.format(msg,float(navdata.batteryPercent),float(navdata.altd), int(navdata.rotZ))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

class MainController(DroneController, VideoDisplay):
	def __init__(self):
		self.pitch = 0
		self.yaw_velocity=0
		self.roll=0
		self.z_velocity=0
		self.rotZ=0
		self.altd=0
		rospy.logwarn('MainController')
		super(MainController,self).__init__()
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 

	def ReceiveNavdata(self,navdata):
		self.altd=navdata.altd
		self.rotZ=navdata.rotZ

	def Takeoff(self):
		print "in main takeoff"
		DroneController().SendTakeoff()
		rospy.logwarn('TAKE-OFF')

	def Land(self):
		print "landing in main"
		DroneController().SendLand()
		rospy.logwarn('Landing')
		rospy.logwarn('Landed Successfully!!!')

	def Forward(self):
		if  controller is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			self.pitch += 1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(2)
			self.pitch -= 1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	def Reverse(self):
		if  controller is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			self.pitch += -1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(2)
			self.pitch -= -1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	def Left(self):
		z = int(self.rotZ)
		if controller is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			self.yaw_velocity +=1 
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(2)
			self.yaw_velocity -=1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
#			if(z>0):
#				if(z<90):
#					while((-5 > (int(self.rotZ)-(z + 85))) and ((int(self.rotZ)-(z + 95)) < 5)):
#						self.yaw_velocity += 1
#				elif(z>90):
#					while((int(self.rotZ))>0):
#						self.yaw_velocity += 1
#					while((z-265)>(int(self.rotZ)) and (int(self.rotZ))<(z-255)):
#						self.yaw_velocity += 1
#			elif(z<0):
#				if(z>-90):
#					while((int(self.rotZ))<0):
#						self.yaw_velocity += 1
#					while((z+85)>(int(self.rotZ)) and (int(self.rotZ))<(z+95)):
#						self.yaw_velocity += 1
#				elif(z<-90):
#					while((int(self.rotZ))<-90):
#						self.yaw_velocity += 1
#					while((z+85)>(int(self.rotZ)) and (int(self.rotZ))<(z+95)):
#						self.yaw_velocity += 1				
#			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
#			self.yaw_velocity -= 1
#			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	def Right(self):
		z = int(self.rotZ)
		if  controller is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			if(z>-90):
				while(int(self.rotZ) != (z-90)):
					self.yaw_velocity += -1
			if(z<-90):
				while(int(self.rotZ) != (((z-90)+360))):
					self.yaw_velocity -= -1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			self.yaw_velocity -= -1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def Up(self):
		if  controller is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			self.z_velocity += 1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(1)
			self.z_velocity -= 1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	def Down(self):
		if  controller is not None:
			self.pitch = 0
			self.roll = 0
			self.yaw_velocity = 0
			self.z_velocity = 0
			self.z_velocity += -1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			time.sleep(1)
			self.z_velocity -= -1
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_ros_drone')
	# Now we construct our Qt Application and associated controllers and windows
	#rospy.init_node('ardrone_image_stream')
	app = QtGui.QApplication(sys.argv)
	display = VideoDisplay()
	controller = DroneController()
	mainc = MainController()
	rospy.logwarn('repeat')
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
	rospy.spin()
	# and only progresses to here once the application has been shutdown
	#rospy.signal_shutdown('Great Flying!')
