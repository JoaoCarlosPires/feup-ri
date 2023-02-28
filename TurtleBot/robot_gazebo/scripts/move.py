#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, cos, sin, inf
from sensor_msgs.msg import LaserScan
import numpy as np

'''
STATES

0	Initial (wait for scan)
1	Go down
2
3
4
5
6
7
8
9
10
11
12
'''

class TurtleBot:
 
	def __init__(self):
		rospy.init_node('turtlebot_node', anonymous=True)
		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10)		
		self.originaltime = rospy.Time.now().to_sec()
		self.state = 0
		self.d = 1
		self.angular = 0
		self.distance = 0
		self.criterium = False
		self.stop = False
		rospy.on_shutdown(self.shutdown)
		
	def move(self):
		rospy.Subscriber('/scan', LaserScan, self.callback_laser)
		vel_msg = Twist()	
		while not rospy.is_shutdown():
			if self.state == 0:
				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
			elif self.state == 1:
				vel_msg.linear.x = 0
				vel_msg.angular.z = self.angular
			elif self.state == 2:
				vel_msg.linear.x = max(abs(self.distance - self.d) / 5, 0.1)
				vel_msg.angular.z = 0
			elif self.state == 3:
				vel_msg.linear.x = 0
				vel_msg.angular.z = self.angular
			elif self.state == 4:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = 0
			elif self.state == 5:
				vel_msg.linear.x = 0
				vel_msg.angular.z = -self.angular
			elif self.state == 6:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = -0.15
			elif self.state == 7:
				vel_msg.linear.x = self.distance / 5
				vel_msg.angular.z = 0.15
			elif self.state == 8:
				vel_msg.linear.x = 0
				vel_msg.angular.z = self.angular
			elif self.state == 9:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = 0
			elif self.state == 10:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = -0.2
			elif self.state == 11:
				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
			elif self.state == 12:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = 0				

			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
		
	def shutdown(self):
		self.velocity_publisher.publish(Twist())
		rospy.sleep(1)
		
	def callback_laser(self, msg):
		laser_range = np.array(msg.ranges)
		self.distance = np.min(laser_range)
		
		if(min(np.min(laser_range[355:360]), np.min(laser_range[0:5])) < np.min(laser_range[310:320]) 
		and np.min(laser_range[265:275]) < np.min(laser_range[310:320])) and not self.criterium:
			print("criterium reached")
			self.state = 12
			print("state = 12")
			self.criterium = True
		
		if self.state == 0:
			self.d = max(np.min(laser_range) * 1/5, 0.7)
			self.state = 1
			return
			
		if self.state == 1:
			directionIndex = np.argmin(laser_range)
			self.angular =  min(abs(360 - directionIndex), directionIndex)/100
			if(directionIndex < 3 or directionIndex > 357):			
				self.state = 2
				print("state = 2")
			return

		if self.state == 2:
			if(np.min(laser_range) <= self.d):
				self.angular =  abs(270 - np.argmin(laser_range))/100
				print("state = 3")
				self.state = 3
			return

		if self.state == 3:
			self.angular =  abs(270 - np.argmin(laser_range))/100
			if(np.argmin(laser_range) < 273 and np.argmin(laser_range) > 267):
				print("state = 4")
				self.state = 4
			return

		if(np.argmin(laser_range) > 350 or np.argmin(laser_range) < 10) and not self.criterium:
			print("state = 2")
			self.state = 2
			return

		if self.state == 4:
			if(laser_range[270] == inf):
				print("state = 5") 
				self.state = 5
				return 

			if(np.min(laser_range) > self.d + 0.1):
				print("state = 6")
				self.state = 6

			if(np.min(laser_range) < self.d - 0.1):
				print("state = 7")
				self.state = 7

			if(self.distance > 2*self.d):
				print("state = 1")
				self.state = 1

			return

		if self.state == 5:
			self.angular =  abs(270 - np.argmin(laser_range))/100
			if(np.argmin(laser_range) < 273 and np.argmin(laser_range) > 267):
				print("state = 6")
				self.state = 6
			return

		if self.state == 6:
			if(np.min(laser_range) <= self.d + 0.05):
				print("state = 4")
				self.state = 4
			return

		if self.state == 7:
			if(np.min(laser_range) >= self.d - 0.05):
				print("state = 4")
				self.state = 4
			return

		if self.state == 12:
			if(np.min(laser_range) <= self.d):
				print("state = 8")
				self.state = 8
			return
		if self.state == 8:
			self.angular =  abs(45 - np.argmin(laser_range[225:315]))/100
			if(laser_range[0] == inf and np.argmin(laser_range[225:315]) < 48 and np.argmin(laser_range[225:315]) > 42):
				print("state = 9")
				self.state = 9	
			return

		if self.state == 9:
			if(np.min(laser_range) <= self.d - 0.3 or np.min(laser_range) > self.d + 0.3):
				print("state = 8")
				self.state = 8
				self.d = np.min(laser_range)
				
			elif(laser_range[270] == inf):
				self.state = 10
				print("state = 10")	

			elif(min(laser_range[285:360] == inf) and self.stop):
				print("stop")
				self.state = 11
			return
		
		if self.state == 10:
			self.angular =  abs(270 - np.argmin(laser_range))/100
			if(np.argmin(laser_range) < 273 and np.argmin(laser_range) > 267):
				print("state = 9")
				self.state = 9
				self.stop = True
			return

if __name__ == '__main__':
	try:
		turtle = TurtleBot()
		turtle.move()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
