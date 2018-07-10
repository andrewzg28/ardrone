#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from drone_estado import DroneStatus
from face.msg import vector

X = 0
Y = 0
outTime = 0
controlTime=100000
COMMAND_PERIOD = 100

class BasicDroneController(object):
	def __init__(self):
		self.status = -1
		self.X = 0
		self.Y = 0
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
		self.vectorData = rospy.Subscriber('/sendVector',vector, self.callback)
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		self.status = navdata.state
		
	def callback(self, data):
		rospy.loginfo("x = %d y = %d" % (data.x, data.y))
		self.X = data.x
		self.Y = data.y

	def SendTakeoff(self):
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		self.pubLand.publish(Empty())
	
	def Autopilot(self,z_velocity=0):
		outTime = 0
		while(outTime <= controlTime):
			if(self.X < 250 and self.X > 0):
				self.command.angular.z = z_velocity=1
				rospy.loginfo('+1')
			if(self.X > 350):
				self.command.angular.z = z_velocity=-1
				rospy.loginfo('-1')
			if(self.X>250 and self.X <350):
				self.command.angular.z = z_velocity=0
				rospy.loginfo('0')
			self.pubCommand.publish(self.command)
			outTime = (int(outTime) + 1)
		rospy.loginfo('Termina Autopiloto')

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
