#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, cos, sin, inf
from sensor_msgs.msg import LaserScan
import numpy as np


# states: 0=initial (wait for scan), 1:go down
#turn until before you is smallest distance
class TurtleBot:
 
	def __init__(self):
		rospy.init_node('turtlebot_node', anonymous=True)
		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10)		
		self.originaltime = rospy.Time.now().to_sec()
		self.section = {'front':0, 'left':0, 'right':0}
		self.state = 0
		self.d = 1
		self.distance = 0
		self.angular = 0
		self.velocity = 0
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
				vel_msg.angular.z = 0.1
			elif self.state == 5:
				vel_msg.linear.x = 0
				vel_msg.angular.z = -self.angular
			elif self.state == 6:
				vel_msg.linear.x = 0.1
				vel_msg.angular.z = -0.1
			elif self.state == 7:
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

		
		if self.state == 0:
			self.d = max(np.min(laser_range) * 1/5, 0.5)
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

		if self.state == 4:
			#if(laser_range[290] == inf):
			#	print("state = 7")
			#	self.state = 7
			#	return
			if(laser_range[270] == inf):
				print("state = 5") 
				self.state = 5
				return 

			if(np.min(laser_range) > self.d + 0.05):
				print("state = 6")
				self.state = 6

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
			#if(laser_range[290] == inf):
		#		print("state = 7")
		#		self.state = 7
		#		return
			if(laser_range[270] == inf):
				print("state = 5") 
				self.state = 5
				return 
			
			if(np.min(laser_range) < self.d - 0.05):
				print("state = 4")
				self.state = 4
			
			if(self.distance > 2*self.d):
				print("state = 1")
				self.state = 1

			return

		if self.state == 7:
			if(laser_range[270] == inf):
				print("state = 5") 
				self.state = 5
			return 
		

		print(len(msg.ranges))
		print(msg.angle_min)
		print(msg.angle_max)
		print(msg.angle_increment)
		
		self.section = {'front': min(laser_range[70:110]), 'left':min(laser_range[0:40]), 'right':min(laser_range[140:180])}
		print(self.section)
		self.action()
		
	def action(self):
		
		return

if __name__ == '__main__':
	try:
		
		turtle = TurtleBot()
		
		
		turtle.move()
			
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
