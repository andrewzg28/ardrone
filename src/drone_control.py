#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from drone_estado import DroneStatus

COMMAND_PERIOD = 100

class BasicDroneController(object):
	def __init__(self):
		self.status = -1
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		self.status = navdata.state

	def SendTakeoff(self):
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		self.pubLand.publish(Empty())
	
	def Autopilot(self,z_velocity=0):
		self.command.angular.z = z_velocity=1
		self.pubCommand.publish(self.command)

	def SendEmergency(self):
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)
