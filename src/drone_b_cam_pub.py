#!/usr/bin/env python3
import cv2
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from drone_cam.msg import drone_feed
from cv_bridge import CvBridge, CvBridgeError
import subprocess

package_directory = subprocess.check_output(["rospack", "find", "drone_cam"]).decode().strip('\n')
filename = package_directory + '/src/office_drone_b.mp4'
cap = cv2.VideoCapture(filename)
frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
frames = 0
bridge = CvBridge()

topic = '/drone_b/camera/image'
pub = rospy.Publisher(topic, drone_feed, queue_size=10)
pubimg = rospy.Publisher(topic + '_Image', Image, queue_size=10)
rospy.init_node('drone2', anonymous=True)
rate = rospy.Rate(30)
img = drone_feed()
imgmsg = Image()

def pubcam():
    global frames, cap, frame_count
    while not rospy.is_shutdown():
        try:
            frames += 1
            if frames == frame_count:
                cap = cv2.VideoCapture(filename)
                frames = 0
            ret, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([30,150,50])
            upper_red = np.array([255,255,180])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            res = cv2.bitwise_and(frame,frame, mask= mask)
            # cv2.imshow('Original',frame)
            edges = cv2.Canny(frame,100,200)
            # cv2.imshow('Edges',edges)
            img.image = bridge.cv2_to_imgmsg(frame, "bgr8")
            img.name = topic
            pub.publish(img)
            imgmsg = img.image
            pubimg.publish(imgmsg)
            rate.sleep()
        except CvBridgeError as e:
            rospy.loginfo(e)

if __name__ == '__main__':
    try:
        pubcam()
    except rospy.ROSInterruptException:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

