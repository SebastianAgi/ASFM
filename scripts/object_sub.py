#!/usr/bin/env python

import rospy
from zed_interface.msg import ObjectsStamped
from zed_interface.msg import Object
import math
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int32
import numpy as np
import time

pub = rospy.Publisher('/sonar_state', Int32, queue_size=1)
points = []

def objectcallback(data):
    object = data.objects
    print(data)


def main():
    rospy.init_node("Subscriber_Node", anonymous=True)
    rospy.Subscriber("objects",ObjectsStamped, objectcallback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
