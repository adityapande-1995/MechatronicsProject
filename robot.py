#!python
from __future__ import print_function
import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import Float64

class Robot:
    def __init__(self):
        # Class variables
        self.zangle = 0 ; self.imu_read_flag = False ; self.rotate_tolerance = 2.5
        # Velocity publisher
        self.pub = rospy.Publisher('cmd_vel', Twist ,queue_size=10)
        rospy.init_node('brain', anonymous=True)
        # Orientation publisher
        self.pub_orientation = rospy.Publisher('z_angle', Float64 ,queue_size=10)
        # IMU subscriber
        rospy.Subscriber("imu",Imu, self._update_imu)
        
    def _update_imu(self, response):
        u = np.array([response.orientation.x, response.orientation.y, response.orientation.z])
        u = u/np.linalg.norm(u)
        theta = 2*np.arccos(response.orientation.w)*180/np.pi
        # Account for +z and -z axis, use +z axis
        if u[2] > 0 : self.zangle = theta
        else : self.zangle = -theta
        # zangle should be between 0 to 360
        if self.zangle > 360 : self.zangle -= 360
        if self.zangle < 0 : self.zangle += 360
        self.imu_read_flag = True
        self.pub_orientation.publish(self.zangle)

    def send_msg(self, vx,vy, omega):
        temp_msg = Twist()
        temp_msg.linear.x = vx ; temp_msg.linear.y = vy ; temp_msg.angular.z = omega
        self.pub.publish(temp_msg)

    # Simple proportional control
    def rotate_by(self, angle):
        while 1:
            if self.imu_read_flag:
                K = (5.0/360) ; target = self.zangle + angle ;  e = target - self.zangle
                # Target should be between 0 and 360.
                if target > 360 : target -= 360
                if target < 0 : target += 360
                while abs(e) > self.rotate_tolerance:
                    e = target - self.zangle
                    signal = e*K
                    self.send_msg(0,0,signal)
                break
            if not self.imu_read_flag:
                print("IMU flag not set")

if __name__ == '__main__':
    pblart = Robot()

    pblart.rotate_by(-45)
    pblart.send_msg(0,0,0)


