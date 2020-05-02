#!python2
from __future__ import print_function
import rospy, time, subprocess
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Bool, String
from nav_msgs.msg import Odometry
from copy import deepcopy

from scipy import optimize, stats
from sys import exit

# Subscribes to /position, /z_angle, /shoot
class Shooter:
	def __init__(self):
		rospy.init_node('shooter', anonymous=True)
		self.ammo = 12
		self.zangle = 0
		self.position = []
		self.fired = False
		rospy.Subscriber("position",Float64MultiArray,self._update_position)
		rospy.Subscriber("z_angle",Float64,self._update_angle)
		rospy.Subscriber("target", String, self._update_target)

	def _update_angle(self, ang):
		self.zangle = ang.data

	def _update_position(self, pos):
		self.position = pos.data

	def _update_target(self,targetdata):
		target_string = targetdata.data
		target_info = target_string.split('|',-1)
		color = target_info[0]
		xPos = float(target_info[1])
		yPos = float(target_info[2])
		
		if abs(xPos-0.5)<0.25 and abs(yPos-0.5)<0.25:
			shoot()

	def shoot(self):
		if self.ammo > 0:
			position = " -x "+ str(self.position[0]) +" -y "+ str(self.position[1]) +" -z " + str(self.position[2] + 2)
			projectile_name = "projectile"+str(self.ammo)
			bashCommand = "rosrun gazebo_ros spawn_model -sdf -file /home/henry/MechatronicsProject/projectile.sdf -model "+ projectile_name + position
			self.ammo=self.ammo-1
			process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
			output, error = process.communicate()
		
	def go(self):
		while True:
			continue
		

if __name__ == '__main__':
	shooter = Shooter()
	shooter.go()



