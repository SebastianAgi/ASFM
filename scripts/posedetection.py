# -*- coding: utf-8 -*-
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

pub = rospy.Publisher('/objects', String, queue_size=1)

def callback(data):
    pub.publish(type(data))

def main():
    rospy.init_node('zed_objects', anonymous=True)
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", String, callback)
    rospy.spin()

if __name__ == '__main__': 
    try:
        main()

    except rospy.ROSInterruptException:
        pass
