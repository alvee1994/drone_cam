#!/usr/bin/env python3
import cv2
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
# from drone_cam.msg import drone_feed
from cv_bridge import CvBridge, CvBridgeError
import subprocess

class viewer:
    def __init__(self, classifier):
        self.package_directory = subprocess.check_output(["rospack", "find", "drone_cam"]).decode().strip('\n')
        self.filename = self.package_directory + '/src/office_drone_b.mp4'
        self.cap = cv2.VideoCapture(self.filename)
        self.frame_count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        self.frames = 0
        self.bridge = CvBridge()

        self.frame_copy = []

        self.topic = '/drone_b/camera/image'
        # self.pub = rospy.Publisher(self.topic, drone_feed, queue_size=10)
        self.pubimg = rospy.Publisher(self.topic + '_Image', Image, queue_size=10)

        self.rate = rospy.Rate(30)
        # self.img = drone_feed()
        self.imgmsg = Image()

        self.classifiers = {
            'haarcascades': cv2.CascadeClassifier('/home/lv/openCV/opencv/data/haarcascades/haarcascade_frontalface_alt.xml'),
            'lbpcascades': cv2.CascadeClassifier('/home/lv/openCV/opencv/data/lbpcascades/lbpcascade_frontalface.xml')
        }

        self.classifier = self.classifiers[classifier]


    def detectFaces(self, frame):
        self.frame_copy = frame.copy()

        grayscale = cv2.cvtColor(self.frame_copy, cv2.COLOR_BGR2GRAY)
        grayscale = cv2.equalizeHist(grayscale)
        faces = self.classifier.detectMultiScale(grayscale, scaleFactor=1.2, minNeighbors=5)

        for (x, y, w, h) in faces:
            cv2.rectangle(self.frame_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)

    def pubcam(self):
        while not rospy.is_shutdown():
            try:
                self.frames += 1
                if self.frames == self.frame_count:
                    self.cap = cv2.VideoCapture(self.filename)
                    self.frames = 0
                ret, frame = self.cap.read()

                self.detectFaces(frame)
                self.imgmsg = self.bridge.cv2_to_imgmsg(self.frame_copy, "bgr8")
                # self.img.name = self.topic
                # self.pub.publish(self.img)

                # self.imgmsg = self.img.image
                self.pubimg.publish(self.imgmsg)
                self.rate.sleep()
            except CvBridgeError as e:
                rospy.loginfo(e)

if __name__ == '__main__':
    data_for_classifying = '/home/lv/openCV/opencv/data'
    rospy.init_node('drone2', anonymous=True)
    publishcam = viewer('lbpcascades')
    try:
        publishcam.pubcam()
    except rospy.ROSInterruptException:
        pass
    finally:
        publishcam.cap.release()
        cv2.destroyAllWindows()

