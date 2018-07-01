#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone')
import rospy

from drone_control import BasicDroneController
from drone_video import DroneVideoDisplay
from PySide import QtCore, QtGui

class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_E
	PitchBackward    = QtCore.Qt.Key.Key_D
	RollLeft         = QtCore.Qt.Key.Key_S
	RollRight        = QtCore.Qt.Key.Key_F
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_T
	Land             = QtCore.Qt.Key.Key_L
	Emergency        = QtCore.Qt.Key.Key_Space
	Autopilot        = QtCore.Qt.Key.Key_M


class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

	def keyPressEvent(self, event):
		key = event.key()

		if controller is not None and not event.isAutoRepeat():
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()
			elif key == KeyMapping.Autopilot:
				controller.Autopilot()
			else:
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1
				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1
				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1
				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1

			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		if controller is not None and not event.isAutoRepeat():
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1
			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1
			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1
			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_keyboard_controller')

	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()

	display.show()

	status = app.exec_()

	rospy.signal_shutdown('Hasta Luego')
	sys.exit(status)