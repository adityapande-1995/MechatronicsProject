#!python2
from __future__ import print_function
import rospy, time, subprocess
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Bool
from nav_msgs.msg import Odometry
from copy import deepcopy

import matplotlib.pyplot as plt
from matplotlib import animation

from scipy import optimize, stats
from sys import exit

def piecewise_linear(x, x0, y0, k1, k2):
    return np.piecewise(x, [x < x0], [lambda x:k1*x + y0-k1*x0, lambda x:k2*x + y0-k2*x0])


# Publishes /shoot, /cmd_vel, /z_angle, /position ; Subscribes to /imu, /odom, /scan
class Robot:
    def __init__(self):
        # Class variables
        self.zangle = 0 ; self.imu_read_flag = False ; self.rotate_tolerance = 1.5
        self.position = [] ; self.position_read_flag = False ; self.ammo = 12
        self.laserscan = None ; self.laserscan_read_flag = False ;
        # Velocity publisher
        self.pub = rospy.Publisher('cmd_vel', Twist ,queue_size=10)
        rospy.init_node('brain', anonymous=True)
        # Orientation publisher
        self.pub_orientation = rospy.Publisher('z_angle', Float64 ,queue_size=10)
		# Shoot publisher
        self.pub_shoot = rospy.Publisher('shoot', Bool, queue_size=10)
        # IMU subscriber
        rospy.Subscriber("imu",Imu, self._update_imu)
        # Odometry pose subscriber
        rospy.Subscriber("odom",Odometry, self._update_position)
        # Position publisher
        self.pub_position = rospy.Publisher('position', Float64MultiArray ,queue_size=10)
        # Laserscan subscriber
        rospy.Subscriber("scan",LaserScan, self._update_laserscan)

    def _update_laserscan(self, resp):
        self.laserscan_read_flag = True ; self.laserscan = resp.ranges ; self.laserscan_random = np.random.randint(1000)

    def _update_position(self, response):
        self.position = [response.pose.pose.position.x, response.pose.pose.position.y, response.pose.pose.position.z]
        self.position_read_flag = True
        temp_msg = Float64MultiArray()
        temp_msg.data = self.position
        self.pub_position.publish(temp_msg)
        
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

    # Simple proportional control, +ve angle roatates towards LEFT
    def rotate_by(self, angle):
        while 1:
            if self.imu_read_flag:
                print("Read IMU !")
                K = (5.0/360) ; target = self.zangle + angle ;  e = target - self.zangle
                # Target should be between 0 and 360.
                if target > 360 : target -= 360
                if target < 0 : target += 360
                while abs(e) > self.rotate_tolerance:

                    e = target - self.zangle
                    signal = e*K
                    self.send_msg(0,0,signal)
                for i in range(5) : self.send_msg(0,0,0)
                break
            if not self.imu_read_flag:
                print("Waiting for IMU reading")

    def make_path(self):
        counter = 0
        sig = self.laserscan_random
        # Init
        left_x = np.array([ 0 for i in range(179,1,-1) ]) ; left_y = np.array([ 0 for i in range(179,1,-1) ])

        # Left half
        lsp = 0.35 ; lfp = 0.6

        while counter < 2: # Take 1 observations of laserscan
            sig2 = self.laserscan_random
            if sig2 != sig:
                sig = sig2
                counter += 1
                left_x = left_x +  np.array([ self.laserscan[i]*np.cos(i*np.pi/180) for i in range(179,1,-1) ]) - lfp
                left_y = left_y +   np.array([ self.laserscan[i]*np.sin(i*np.pi/180) for i in range(179,1,-1) ]) - lsp

        left_x = left_x/counter ; left_y = left_y/counter 

        ld_x = [ left_x[i+1] - left_x[i] for i in range(len(left_x) -1 )  ]
        ld_y = [ left_y[i+1] - left_y[i] for i in range(len(left_y) -1 )  ]
        # derivatives
        left_d = np.array([ [x,y] for x,y in zip(ld_x,ld_y) ])
        # Closest pt to origin with -ve x
        temp_array = []
        for x,y in zip(left_x, left_y):
            if x < 0 :
                temp_array.append(x**2 + y**2)
            else :
                temp_array.append(1000)

        l_min = np.argmin(temp_array)

        return [left_x, left_y,l_min,left_d]

    def wall_follow(self):
        L = self.make_path()

        r = 5 ; l_min = L[2] ; 
        # running distance ratio
        distances = [ np.linalg.norm(t) for t in L[3][l_min -r : l_min + r] ]
        dist_ratio = [ distances[i+1]/distances[i] for i in range(len(distances) - 1) ]
        L_mdr = max(dist_ratio)

        print("Left max dist ratio : ",L_mdr )
        
        # Left
        p , e = optimize.curve_fit(piecewise_linear, np.nan_to_num(L[0][l_min -r : l_min + r]),np.nan_to_num( L[1][l_min -r : l_min + r]) )
        # Find angle
        temp_x =[ L[0][l_min + r - 1] , L[0][l_min + r] ]
        temp_y = piecewise_linear(temp_x, *p)
        v = [ temp_x[1] - temp_x[0] , temp_y[1] - temp_y[0] ]
        angle = np.arctan( v[1]/v[0] )*180/np.pi
        offset = (L[0][l_min])**2 + piecewise_linear(L[0][l_min], *p)**2
        offset_dist = offset**0.5
        if L[1][l_min] < 0:
            offset_dist = -1*offset_dist
        print("Left -- angle, offset = ", angle, offset_dist)
        v = (np.array(v)/np.linalg.norm(v))*0.2
        self.ax.arrow(0,0,v[0], v[1],color="blue")
        xd = L[0][l_min -r : l_min + r]

        L_angle = angle ; L_od = offset_dist

        # Gen path plotting
        self.ax.scatter(L[0], L[1],s=1, label="Left", color="black")
        self.ax.text(L[0][0], L[1][0], s="179 deg L")
        self.ax.text(L[0][-1], L[1][-1], s="1 deg L")
        self.ax.scatter([0],[0], color="red")

        if L_mdr < 5 :
            return L_angle, L_od
        elif self.laserscan[90] < 0.8:
            print("About to lose left wall")
            return 0,0
        else: # Some weird situation
            return 100,100

    def align_with_wall(self):
        counter = 0
        sig = self.laserscan_random
        x = [ 0 for i in range(90-5, 90+6)] ; y = [ 0 for i in range(90-5, 90+6)]
        while counter < 5:
            sig2 = self.laserscan_random
            if sig != sig2:
                sig = sig2
                counter += 1
                x = x + [ self.laserscan[i]*np.cos((i)*np.pi/180) for i in range(90-5, 90+6)]
                y = y + [ self.laserscan[i]*np.sin((i)*np.pi/180) for i in range(90-5, 90+6)]

        x = np.array(x)/counter ; y = np.array(y)/counter
        angle = np.arctan(stats.linregress(x,y)[0])*180/np.pi

        print("Angle to align with left wall : ", angle)
        self.rotate_by(angle)

    def animate_cb(self,m):
        # Plotting init
        self.ax.clear()
        self.ax.grid()

        # Start following wall
        angle, offset = self.wall_follow()

        if angle != 100 and angle != 0  and offset != 100 and offset != 0:
            # Simple wall folowing case
            if angle > 15 or angle < -15: # If angle is too off
                print("Rotating by ", angle)
                self.rotate_by(angle)
            else:
                Kr = 0.5
                self.send_msg(0.15,0, Kr*offset)

        # Wall broken, Left turn sequence
        elif angle == 0 and offset == 0 :
            # Left turn sequence
            self.align_with_wall()
            while self.laserscan[100] < 0.7 : self.send_msg(0.15,0,0)
            self.rotate_by(90)
            print("Finding wall..")
            while self.laserscan[95] > 0.7 : self.send_msg(0.15,0,0)
            self.send_msg(0,0,0)
            print("Found wall")
            self.align_with_wall()

        else :
            print("Weird case found")
            while self.laserscan[90] > 0.8 : self.send_msg(0.15,0,0)

        self.ax.legend()


    def go(self):
        while 1:
            if self.laserscan_read_flag: break
            else : print("Waiting for laserscan")

        print("Read laserscan")

        self.fig = plt.figure() ; self.ax = plt.axes()
        plt.axis('scaled')
        self.anim_object = animation.FuncAnimation(self.fig, self.animate_cb)
        plt.show()


if __name__ == '__main__':
    pblart = Robot()
    pblart.rotate_by(-90)
    pblart.go()

