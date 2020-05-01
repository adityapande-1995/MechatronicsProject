#!python2
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import numpy as np

class TargetDetector:
    def __init__(self):
        rospy.init_node('target_detector', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw", Image, self.callback_rgb)
        rospy.Subscriber("camera/depth/image_raw", Image, self.callback_depth)
        # Flags
        self.flag_rgb = False ; self.flag_depth = False;

    def callback_rgb(self,resp):
        self.raw_rgb = self.bridge.imgmsg_to_cv2(resp, desired_encoding="bgr8")
        self.flag_rgb = True

    def callback_depth(self, resp):
        self.raw_depth = self.bridge.imgmsg_to_cv2(resp, desired_encoding="passthrough")
        self.flag_depth = True

    def find_target(self):
        while 1:
            if not (self.flag_rgb and self.flag_depth) :
                print("Waiting to receive depth and rgb frames")
                time.sleep(1)
            else:
                print("Read depth and rgb frames")
                break

        # Read frames, start processing
        low_blue = np.array([30, 0, 0]) ; high_blue = np.array([125, 5, 5])
        low_green = np.array([0, 30, 0]) ; high_green = np.array([5, 50, 5])
        low_red = np.array([0, 0, 25]) ; high_red = np.array([20, 20, 100])
        while True:
            # Get masks
            print("-------- Processing frame -------")
            red_mask = cv2.inRange(self.raw_rgb.copy(), low_red, high_red)
            green_mask = cv2.inRange(self.raw_rgb.copy(), low_green, high_green)
            blue_mask = cv2.inRange(self.raw_rgb.copy(), low_blue, high_blue)

            # Find largest contour and draw
            findings = []
            for i,m in enumerate([blue_mask, green_mask, red_mask]):
                contours = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                if len(contours) != 0:
                    c = max(contours, key = cv2.contourArea)
                    if cv2.contourArea(c) > 1000 : findings.append((i,c.copy()))

            img = self.raw_rgb.copy()
            for i,cnt in findings:
                if i == 0 : print("Found blue target")
                elif i == 1 : print("Found green target")
                elif i == 2 : print("Found red target")
                print("Area : ", cv2.contourArea(cnt))
                img = cv2.drawContours(img, [cnt], 0, (255,255,255), 3)

            # Show image
            #masks = np.hstack((blue_mask, green_mask ))
            #masks = np.hstack((masks, red_mask))
            #masks = cv2.resize(masks, (0,0), fx=0.2, fy=0.2)
            #cv2.imshow("Masks", masks)
            img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
            cv2.imshow("Overlay", img)
            key = cv2.waitKey(1)
            if key == 27:
                break

if __name__ == '__main__':
    T = TargetDetector()
    T.find_target()


