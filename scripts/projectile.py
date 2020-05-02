#!python2
from __future__ import print_function
import rospy, time, subprocess
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from copy import deepcopy
import os

from scipy import optimize, stats
from sys import exit

# Subscribes to /position, /z_angle, /shoot
class Shooter:
	def __init__(self):
		rospy.init_node('shooter', anonymous=True)
		self.ammo = 12
		self.zangle = 0
		self.position = [0,0,0]
		self.fired = False
		rospy.Subscriber("odom",Odometry, self._update_position)
		rospy.Subscriber("target", String, self._update_target)
		rospy.Subscriber("imu",Imu, self._update_imu)



	def _update_position(self, response):
		self.position = [response.pose.pose.position.x, response.pose.pose.position.y, response.pose.pose.position.z]

	def _update_imu(self, response):
		u = np.array([response.orientation.x, response.orientation.y, response.orientation.z])
		u = u/np.linalg.norm(u)
		theta = 2*np.arccos(response.orientation.w)*180/np.pi
		if u[2] > 0 : self.zangle = theta
		else : self.zangle = -theta
		if self.zangle > 360 : self.zangle -= 360
		if self.zangle < 0 : self.zangle += 360

	def _update_target(self,targetdata):
		target_string = targetdata.data
		target_info = target_string.split('|',-1)
		color = target_info[0]
		xPos = float(target_info[1])
		yPos = float(target_info[2])
		
		if abs(xPos-0.5)<0.25 and abs(yPos-0.5)<0.25:
			self.shoot()

	def shoot(self):
		print("shooting")
		if self.ammo > 0:
			position = " -x "+ str(self.position[0]) +" -y "+ str(self.position[1]) +" -z " + str(self.position[2] + 0.5)
			projectile_path = os.getcwd() + "/maps/projectile.sdf"
			projectile_name = "projectile"+str(13-self.ammo)
			bashCommand = "rosrun gazebo_ros spawn_model -sdf -file " +projectile_path + " -model "+ projectile_name + position
			self.runBash(bashCommand)
			self.ammo=self.ammo-1
			if self.ammo<11:
				#delete previous projectile
				bashCommand = "rosservice call gazebo/delete_model '{model_name: projectile"+str(13-self.ammo+2)+"}'"
				self.runBash(bashCommand)
				
			
		
	def go(self):
		while True:
			continue

	def runBash(self,bashCommand):
		process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
		output, error = process.communicate()
	
		

if __name__ == '__main__':
	shooter = Shooter()
	shooter.go()



