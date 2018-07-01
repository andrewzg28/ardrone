#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone')
import rospy

from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from threading import Lock
from drone_estado import DroneStatus
from PySide import QtCore, QtGui


CONNECTION_CHECK_PERIOD = 250
GUI_UPDATE_PERIOD = 20
DETECT_RADIUS = 5

class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergencia',
		DroneStatus.Inited    : 'Inicializa',
		DroneStatus.Landed    : 'En Tierra',
		DroneStatus.Flying    : 'Volando',
		DroneStatus.Hovering  : 'Flotando',
		DroneStatus.Test      : 'Test',
		DroneStatus.TakingOff : 'Despegando',
		DroneStatus.GotoHover : 'Cambio de Modo',
		DroneStatus.Landing   : 'Aterrizando',
		DroneStatus.Looping   : 'Loop',
		DroneStatus.Autopilot : 'Autopiloto'
		}
	DisconnectedMessage = 'Desconectado'
	UnknownMessage = 'Desconocido'
	
	def __init__(self):
		super(DroneVideoDisplay, self).__init__()

		self.setWindowTitle('AR.Drone Buscador AutoPiloto')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
		
		self.statusMessage = ''

		self.communicationSinceTimer = False
		self.connected = False

		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)

	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			self.imageLock.acquire()
			try:			
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(255,255,0))
							painter.setBrush(QtGui.QColor(255,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
			finally:
				self.imageLock.release()

			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		self.communicationSinceTimer = True

		self.imageLock.acquire()
		try:
			self.image = data
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		self.communicationSinceTimer = True

		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Bateria: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Hasta Luego')
	sys.exit(status)
