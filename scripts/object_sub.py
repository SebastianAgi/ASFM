#!/usr/bin/env python3
import zed_interfaces.msg
import geometry_msgs.msg
import std_msgs.msg
import rospy
import math
import message_filters
import numpy as np

pub = rospy.Publisher('/obj_too_close', zed_interfaces.msg.FloatList, queue_size=10)
warn_info = zed_interfaces.msg.FloatList()

def ellipse_center_dist(angle, a, b):
    # x = (a*b)/math.sqrt(b**2 + (a**2)*math.tan(angle)**2)
    # y = x*math.tan(angle)
    x = a*math.sin(math.pi/2 - angle)
    y = b*math.cos(math.pi/2 - angle)
    dist = math.sqrt(x**2 + y**2)
    return dist

def distance(x,y):
    return math.sqrt(x**2 + y**2)

def callback(pose, obj_det): 
     
    for item in obj_det.objects:
        if (item.label_id == -1):
            continue

        log_ = "***** Human %s *****"%item.label_id
        rospy.loginfo(log_)

        # distance = item.position[0]
        # if distance <= 2:
        #     rospy.loginfo("Human too close!")

        #Spot Ellipse major and minor axis:
        spot_el_a = 1.5
        spot_el_b = 1

        #Position of detected body
        current_human_pos = "\n(X,Y): ({},{})".format(round(item.position[0],3),round(item.position[1],3))

        current_human_dist = distance(item.position[0], item.position[1])

        #Angle of detected human relative to camera (zero degtrees in front of camera)
        angle = math.atan2(item.position[1],item.position[0])

        #Forbidden ellipse length infront of detected human
        length = ellipse_center_dist(angle, spot_el_a, spot_el_b)

        if current_human_dist <= length:
            warn_info = [length-current_human_dist, angle]
            # rospy.loginfo("Human within safety area")
            # rospy.loginfo("move back {}".format(warn_info[0]))
            pub.publish(zed_interfaces.msg.FloatList(warn_info))

        velo = "(vx,vy,vz): [{},{},{}]".format(round(item.velocity[0],2),round(item.velocity[1],2),round(item.velocity[2],2))
        

        

    # rospy.loginfo(pose.pose)

def main():
    pose_sub = message_filters.Subscriber('/zed2/zed_node/pose', geometry_msgs.msg.PoseStamped)
    object_sub = message_filters.Subscriber('/zed2/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped)

    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, object_sub],queue_size = 10, slop = 0.5)
    ts.registerCallback(callback)

    # rospy.Subscriber('/zed2/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped, callback)
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("Subscriber_Node", anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
