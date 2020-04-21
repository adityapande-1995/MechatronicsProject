#!python2
from __future__ import print_function
import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from copy import deepcopy

# Publishes /cmd_vel, /z_angle, /position ; Subscribes to /imu, /odom, /scan
class Robot:
    def __init__(self):
        # Class variables
        self.zangle = 0 ; self.imu_read_flag = False ; self.rotate_tolerance = 1.5
        self.position = [] ; self.position_read_flag = False ;
        self.laserscan = None ; self.laserscan_read_flag = False ;
        # Velocity publisher
        self.pub = rospy.Publisher('cmd_vel', Twist ,queue_size=10)
        rospy.init_node('brain', anonymous=True)
        # Orientation publisher
        self.pub_orientation = rospy.Publisher('z_angle', Float64 ,queue_size=10)
        # IMU subscriber
        rospy.Subscriber("imu",Imu, self._update_imu)
        # Odometry pose subscriber
        rospy.Subscriber("odom",Odometry, self._update_position)
        # Position publisher
        self.pub_position = rospy.Publisher('position', Float64MultiArray ,queue_size=10)
        # Laserscan subscriber
        rospy.Subscriber("scan",LaserScan, self._update_laserscan)

    def _update_laserscan(self, resp):
        self.laserscan_read_flag = True ; self.laserscan = resp.ranges

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
                K = (5.0/360) ; target = self.zangle + angle ;  e = target - self.zangle
                # Target should be between 0 and 360.
                if target > 360 : target -= 360
                if target < 0 : target += 360
                while abs(e) > self.rotate_tolerance:

                    e = target - self.zangle
                    signal = e*K
                    self.send_msg(0,0,signal)
                self.send_msg(0,0,0)
                break
            if not self.imu_read_flag:
                print("Waiting for IMU reading")


    def align_with_wall(self, left=0):
        while 1:
            if self.laserscan_read_flag: break
            else : print("Waiting for laserscan")

        angle_step = 0.0175019223243
        angles = np.arange(0,2*np.pi, angle_step)*180/np.pi
        if not left : req_angle = angles[ np.argmin(self.laserscan) ]
        else : req_angle = angles[ 70+ np.argmin(self.laserscan[70:110]) ]
        print("Explore mode : Aligning with the wall")
        self.rotate_by(req_angle - 90 )
        # angles[90] = 90.75 deg ; angles[270] = 270.75 deg
        print("Distances on left and right: ",self.laserscan[90], self.laserscan[270])

    def move_ahead(self):
        # Keep tracking left wall
        while True : 
            lf = self.laserscan[70] ; lm = self.laserscan[90] ; lb = self.laserscan[110]
            front_dist = min(self.laserscan[-2 :] + self.laserscan[ : 2])
            wall_thresh_close = 0.23 ; wall_thresh_far = 0.27
            ang_vel = 0.1 ; angle_thresh = 0.03; lin_vel = 0.15
            print("lf-lb = ",lf-lb, " lm = ", lm, "front_dist = ", front_dist)
            if front_dist < 0.35:
                print("Obstacle ahead, stop")
                self.send_msg(0,0,0)
                break
            # No obstacle ahead
            elif lf - lb > angle_thresh and lm < wall_thresh_close : 
                print("Pointing right, too close to wall, go straight")
                self.send_msg(lin_vel,0,0)
            elif lf - lb > angle_thresh and lm > wall_thresh_far :
                print("Pointing right, too far away from wall, steering left !")
                self.send_msg(lin_vel,0,ang_vel)
            elif lb - lf > angle_thresh and lm > wall_thresh_far: # Pointing towards left but too far
                print("Pointing left, too far from wall, go straight")
                self.send_msg(lin_vel,0,0)
            elif lb - lf > angle_thresh and lm < wall_thresh_close:
                print("Pointing left, too close to wall, steer right !")
                self.send_msg(lin_vel,0,-ang_vel)
            elif lf - lb < angle_thresh and lf - lb > -angle_thresh :
                if lm < wall_thresh_close:
                    print("Straight but too close to wall, steer right")
                    self.send_msg(lin_vel,0,-ang_vel)
                elif lm > wall_thresh_far:
                    print("Straight but too far from wall, steer left")
                    self.send_msg(lin_vel,0,ang_vel)
                else :
                    print("Straight and midway, go straight")
                    self.send_msg(lin_vel,0,0)
            else :
                print("Weird case, go straight")
                self.send_msg(0.1,0,0)

    # Explore mode
    def explore(self):
        self.align_with_wall()
        while 1 :
            self.move_ahead()
            # Move ahead exited, obstacle ahead
            side_clearance = self.laserscan[90] - self.laserscan[270]
            back_clearance = self.laserscan[180]
            print(side_clearance, back_clearance)
            if side_clearance > 0.3 :
                print("Rotating left") 
                self.rotate_by(90)
            elif side_clearance < -0.3: 
                print("Rotating right")
                self.rotate_by(-90) ;
            elif back_clearance > 0.5 :
                print("Turning around") 
                self.rotate_by(180)
            else:
                print("Weird case found in explore, exiting")
                break

            # print("Realigning wrt new wall")
            # self.align_with_wall(1)


if __name__ == '__main__':
    pblart = Robot() 
    pblart.explore()
    # pblart.rotate_by(-45)


