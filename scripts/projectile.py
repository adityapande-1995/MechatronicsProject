#!python2
from __future__ import print_function
import rospy, time, subprocess
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Bool, String
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteModel, SpawnModel, ApplyBodyWrench, SetLinkState
from gazebo_msgs.msg import LinkState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, Vector3, Twist
from copy import deepcopy
import os

from scipy import optimize, stats
from sys import exit

# Subscribes to /position, /z_angle, /shoot
class Shooter:
	def __init__(self):
		rospy.init_node('shooter', anonymous=True)
		self.has_fired = False
		self.time_last_shot = rospy.get_time()
		self.ammo = 12
		self.zangle = 0
		self.position = [0,0,0]
		self.last_projectile_deleted = True
		rospy.Subscriber("odom",Odometry, self._update_position)
		rospy.Subscriber("imu",Imu, self._update_imu)
		rospy.Subscriber("target", String, self._update_target)



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
		if color.lower() != "red":
			#we only want to shoot red
			return
		if abs(xPos-0.5)<0.25 and abs(yPos-0.5)<0.25:
			if rospy.get_time()-self.time_last_shot>15 and self.has_fired:
				self.time_last_shot = rospy.get_time()
				self.shoot()
				self.last_projectile_deleted = False
			elif rospy.get_time()-self.time_last_shot>10 and self.last_projectile_deleted == False and self.has_fired:
				self.delete_shot()
				self.last_projectile_deleted = True
			elif rospy.get_time()-self.time_last_shot>15 and self.has_fired == False:
				self.has_fired = True


	def shoot(self):
		print("Shooting")
		if self.ammo > 0:
			#spawn projectile
			rospy.wait_for_service("gazebo/spawn_sdf_model")
			spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model",SpawnModel)
			projectile_path = os.getcwd() + "/maps/projectile.sdf"
			projectile_name = "projectile"+str(13-self.ammo)
			pose = Pose()
			posePos = Point()
			posePos.x = self.position[0]
			posePos.y = self.position[1]
			posePos.z = self.position[2]
			poseAng = Quaternion()
			poseAng.x = 0; poseAng.y = 0; poseAng.z = 0; poseAng.w = 1
			pose.position = posePos
			pose.orientation = poseAng
			pose.position.z += 0.25

			f = open(projectile_path)
			sdf = f.read()
			
			spawn(projectile_name, sdf, "", pose,"")
			link_name = projectile_name+"::projectile_link"
			twist = Twist()
			velocity = 3.0
			twist.linear.x = velocity*np.cos(self.zangle*np.pi/180)
			twist.linear.y = velocity*np.sin(self.zangle*np.pi/180)
			ls = LinkState()
			ls.link_name = link_name; ls.pose = pose; ls.twist = twist
			rospy.wait_for_service("gazebo/set_link_state")
			set_state = rospy.ServiceProxy("gazebo/set_link_state",SetLinkState)
			set_state(ls)
			self.ammo=self.ammo-1
		
	def delete_shot(self):
		#delete previous projectile
		print("Deleting last shot")
		rospy.wait_for_service("gazebo/delete_model")
		delete = rospy.ServiceProxy("gazebo/delete_model",DeleteModel)
		delete("projectile"+str(13-(self.ammo+1)))		
			
		
	def go(self):
		while True:
			continue
	
		

if __name__ == '__main__':
	shooter = Shooter()
	shooter.go()



