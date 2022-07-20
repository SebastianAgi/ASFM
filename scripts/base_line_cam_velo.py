#!/usr/bin/env python
import math
import rospy
import sensor_msgs
import geometry_msgs
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int64
import numpy as np
import time

pub = rospy.Publisher('/zed2/zed_node/self_velocity', geometry_msgs.msg.TwistStamped, queue_size=1)
twist = geometry_msgs.msg.TwistStamped()
x = 0
y = 0
prev_x = 0
prev_y = 0
time_past = 0
current_time = 0

def l2norm(x,y):
    return math.sqrt(x**2 + y**2)

def callback(data):
    global twist, x, y, prev_x, prev_y, time_past, current_time

    prev_x = x
    prev_y = y
    x = data.pose.position.x
    y = data.pose.position.y

    twist.twist.linear.x = x - prev_x
    twist.twist.linear.y = y - prev_y
    twist.header.stamp = data.header.stamp
    pub.publish(twist)


def main():
    rospy.init_node("Subscriber_Node", anonymous=True)
    rospy.Subscriber('/zed2i/zed_node/pose', geometry_msgs.msg.PoseStamped, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
