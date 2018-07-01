#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone')
import rospy

from drone_control import BasicDroneController
from drone_video import DroneVideoDisplay
from sensor_msgs.msg import Joy
from PySide import QtCore, QtGui

ButtonEmergency = 0
ButtonLand      = 1
ButtonTakeoff   = 2
ButtonAutopilot = 3

AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
AxisZ           = 4

ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

def ReceiveJoystickMessage(data):
	if data.buttons[ButtonEmergency]==1:
		rospy.loginfo("Boton de Emergencia")
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Boton de Aterrizaje")
		controller.SendLand()
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Boton de Despege")
		controller.SendTakeoff()
	elif data.buttons[ButtonAutopilot]==1:
		rospy.loginfo("Boton de Autopiloto")
		controller.Autopilot()
	else:
		controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw,data.axes[AxisZ]/ScaleZ)

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_joystick_controller')

	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	ButtonAutopilot = int (   rospy.get_param("~ButtonAutopilot",ButtonAutopilot) )
	AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
	AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
	AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
	AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
	ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
	ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
	ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
	ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )

	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	controller = BasicDroneController()

	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)

	display.show()
	status = app.exec_()

	rospy.signal_shutdown('Hasta Luego')
	sys.exit(status)