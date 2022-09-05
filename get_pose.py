#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_driver.msg
import spot_driver.srv

import argparse
import time
import math
import sys
import readchar
import os
# import json

line='\u2500'
instructions="\n\
\u250C{}   SPOT WAY PLANNER   {}\u2510 \n\
\u2502                            \u2502\n\
\u2502     t - Record pose        \u2502\n\
\u2502     u - Save pose(s)       \u2502\n\
\u2502     0 - start pose         \u2502\n\
\u2502     1 - Pose 1             \u2502\n\
\u2502     2 - Pose 2             \u2502\n\
\u2502     3 - Pose 3             \u2502\n\
\u2502     4 - Pose 4             \u2502\n\
\u2502     5 - Pose 5             \u2502\n\
\u2502     6 - Pose 6             \u2502\n\
\u2502     7 - Pose 7             \u2502\n\
\u2502     8 - Pose 8             \u2502\n\
\u2502     9 - Pose 1 to 8        \u2502\n\
\u2502                            \u2502\n\
\u2514{}\u2518\
".format(line*3,line*3,line*28)

# Get size of terminal window
rows, columns = os.popen('stty size', 'r').read().split()

pose_file_num = 1
path = '/catkin_ws/src' #Directory to save Pose file
save_file_name = "Pose_"+str(pose_file_num)+".txt"
load_file_name = "Pose_load.txt"
completeName_save = os.path.join(path, save_file_name)
completeName_load = os.path.join(path, load_file_name)
num = 0
coord = geometry_msgs.msg.Pose()
new_coord = geometry_msgs.msg.Pose()
key_pressed = None


class Pose_record:

    def __init__(self):

        # rospy.init_node('Pose_record')
        self.rate = rospy.Rate(5)  #Standard Boston Dynamics rate
        self.Posesx = []
        self.file_list = []
        self.num = 1
        self.pose_file = open(completeName_load, "r")
        rospy.loginfo(completeName_load)
        self.key_pressed = None
        for line in self.pose_file:
              self.stripped_line = line.strip()
              self.line_list = self.stripped_line.split()
              self.file_list.append(self.line_list)

        #Pre-load pose coordinates
        self.Poses = []
        for i in range(9):
            self.pose = geometry_msgs.msg.Pose()
            self.pose.position.x = float(self.file_list[3+i*12][1])
            self.pose.position.y = float(self.file_list[4+i*12][1])
            self.pose.position.z = float(self.file_list[5+i*12][1])
            self.pose.orientation.x = float(self.file_list[7+i*12][1])
            self.pose.orientation.y = float(self.file_list[8+i*12][1])
            self.pose.orientation.z = float(self.file_list[9+i*12][1])
            self.pose.orientation.w = float(self.file_list[10+i*12][1])
            self.Poses.append(self.pose)

        #Service Proxies
        self.pose_srv_pub = rospy.ServiceProxy("pose_cmd", spot_driver.srv.GetPose)
        self.trajectory_srv_pub = rospy.ServiceProxy("trajectory_cmd", spot_driver.srv.Trajectory)
        self.duration_trajectory_srv_pub = rospy.ServiceProxy("duration_trajectory_cmd", spot_driver.srv.Trajectory_duration)
        self.vel_srv_pub = rospy.ServiceProxy("velocity_cmd", spot_driver.srv.Velocity)

        #Sevice Requests 
        self.pose_srv_req = spot_driver.srv.GetPoseRequest()
        self.trajectory_srv_req = spot_driver.srv.TrajectoryRequest()
        self.duration_trajectory_srv_req = spot_driver.srv.Trajectory_durationRequest()
        self.vel_srv_req = spot_driver.srv.VelocityRequest()

        print(instructions)        

    def pose_service(self, key):

        if key=='t':
            try:
                rospy.wait_for_service("pose_cmd", timeout=2.0)
                temp = 'Pose ' + str(self.num) + '\n' + str(self.pose_srv_pub()) + '\n'
                print(temp)
                self.Posesx.append(temp)
                self.num += 1
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e, end='')

        if key=='c':
            try:
                rospy.wait_for_service("pose_cmd", timeout=2.0)
                print(self.pose_srv_pub())
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e, end='')

        elif key=='u':
            with open(completeName_save, "w") as outfile: #needs full path directory
                for item in self.Posesx:
                    outfile.write("%s\n" % item)
                print('Saved poses')
        
        elif key==55:
            rospy.wait_for_service("pose_cmd", timeout=2.0)
            temp = self.pose_srv_pub()
            return temp

    def trajectory_service(self, key, time, *order):
        posex = geometry_msgs.msg.Pose()
        twist = geometry_msgs.msg.Twist()
        flt = std_msgs.msg.Float64()
        # # dur = []
        # dur = std_msgs.msg.Float32()
        if not order:
            flt = time
            self.key_pressed = key
            if key=='1':
                posex = self.Poses[0]
            elif key=='2':
                posex = self.Poses[1]
            elif key=='3':
                posex = self.Poses[2]
            elif key=='4':
                posex = self.Poses[3]
            elif key=='5':
                posex = self.Poses[4]
            elif key=='6':
                posex = self.Poses[5]
            elif key=='7':
                posex = self.Poses[6]
            elif key=='8':
                posex = self.Poses[7]
            elif key=='0':
                posex = self.Poses[8]
            elif key=='9':
                posex = self.Poses

            self.duration_trajectory_srv_req.waypoints = posex
            self.duration_trajectory_srv_req.duration.data = flt

            try:
                rospy.wait_for_service("duration_trajectory_cmd", timeout=2.0)
                self.duration_trajectory_srv_pub(self.duration_trajectory_srv_req)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e, end='')

        else:
            temp = order[0]
            twist.linear.x = temp.data[0]
            twist.linear.y = temp.data[1]
            twist.angular.z = temp.data[5]
            
            self.vel_srv_req.velocity.linear = twist.linear
            self.vel_srv_req.velocity.angular = twist.angular

            try:
                rospy.wait_for_service("velocity_cmd", timeout=2.0)
                self.vel_srv_pub(self.vel_srv_req)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e, end='')


class Human_avoidance:

    def __init__(self, Pose_record):
        rospy.Subscriber('/obj_too_close', spot_driver.msg.FloatList, self.callback)
        self.mydata = spot_driver.msg.FloatList()

        self.pose_srv = Pose_record

    def callback(self, msg):
        self.mydata = msg
        time = self.mydata.data[4]

        if self.mydata.data[2] == 0:
            self.pose_srv.trajectory_service(55, time, self.mydata)
        else:
            self.pose_srv.trajectory_service(str(int(self.mydata.data[3])),time)

        
if __name__ == "__main__":
    rospy.init_node("Subscriber_Node", anonymous=True)
    parser = argparse.ArgumentParser()
    options, unknown = parser.parse_known_args(sys.argv[1:])
    print(unknown)
    
    Pose_record_ros = Pose_record() 
    Human_avoidance_ros = Human_avoidance(Pose_record_ros)
    
    while not rospy.is_shutdown():

        key = readchar.readkey()
        print('{}\rKey pressed: {}\n'.format(' '*int(columns), key), end="")

        if key=="Q":
            sys.exit()
        if key in 'tuc':
            Pose_record_ros.pose_service(key)
        elif key in '1234567890':
            Pose_record_ros.trajectory_service(key, 5.5)

        Pose_record_ros.rate.sleep()
