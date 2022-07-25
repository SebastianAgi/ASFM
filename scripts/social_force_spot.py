#!/usr/bin/env python3
import zed_interfaces.msg
import geometry_msgs.msg
import std_msgs.msg
import rospy
import time
import math
import message_filters
import numpy as np
from numpy import *
import spot_driver.msg

pub = rospy.Publisher('/obj_too_close', zed_interfaces.msg.FloatList, queue_size=10)
velo = zed_interfaces.msg.FloatList()
flag = False
resume_checkpoint = 0
past_time = 0
current_time = 0
current_checkpoint = 0

path = '/ros_ws/src' #Directory to save Pose file
load_file_name = "Pose_load.txt"
completeName_load = os.path.join(path, load_file_name)

self.pose_file = open(completeName_load, "r")
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

def check_next_pose(x, y):
    current_checkpoint = 0 
    ##########################################
    # Add check for what checkopoint to follow
    # add the pose file for the pose list
    # finish model
    ##########################################

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def desired_force(desired_direction, vmax, current_velo, RelaxationTime):
    force = (desired_direction * vmax - current_velo) / RelaxationTime
    return force

#spot is an np array (x,y vector), ped is obj_det_.objects
def social_force(spot, ped):
    force = 0
    lambda_ = 1.0
    A = 5.5
    B = -2
    C = 1.7
    
    for i in range(len(ped.objects)):

        diff = np.array([[ped.objects[i].position[0]-spot[0,0]], 
                         [ped.objects[i].position[1]-spot[1,0]]])

        diffDirection = unit_vector(diff)
        diffDirection = np.reshape(diffDirection, (2,1))

        if ped.objects[i].action_state == 0:
            ped_velo = np.array([[0],
                                 [0]])
        else:
            ped_velo = np.array([[ped.objects[i].velocity[0]],
                                 [ped.objects[i].velocity[1]]])

        velDiff = spot - ped_velo

        interactionVector = np.add(lambda_ * velDiff, diff)
        
        repulsive_force = A*math.exp(B*np.linalg.norm(interactionVector) + C) # Ae^(Bx + C)
        
        force = np.array([[diffDirection[0,0]*repulsive_force],
                          [diffDirection[1,0]*repulsive_force]])

    return force

def move(delta_time,socialforce, desiredforce, current_velo):

    a = np.add(socialforce,desiredforce)
    v = current_velo + delta_time * a

    return v


def callback(spot, obj_det):
    global past_time, current_time, velo, flag
    RelaxationTime = 0.4
    v_max = 1.0
    v_rotate = 0.5
    goal_x = -4.902107704062697
    goal_y = -18.595383951711007
    threshold = 0.2

    if flag == False:
        past_time = rospy.Time.now()
        flag = True

    spot_velo = np.array([[spot.velocity_of_body_in_odom.linear.x],
                          [spot.velocity_of_body_in_odom.linear.y]])

    direction = np.array([[goal_x - spot.vision_tform_body.translation.x],
                          [goal_y - spot.vision_tform_body.translation.y]])

    desired_direction = unit_vector(direction)

    g_force = desired_force(desired_direction, v_max, spot_velo, RelaxationTime)

    s_force = social_force(spot_velo, obj_det)

    if np.linalg.norm(s_force) < threshold:
        resume_checkpoint = 1
    else:
        resume_checkpoint = 0

    current_time = rospy.Time.now()
    delta_time = float(rospy.Time.__str__(rospy.Time.__sub__(current_time,past_time)))/10**9
    
    new_spot_velo = move(delta_time,s_force, g_force, spot_velo)
    velo.linear.x = new_spot_velo[0,0]
    velo.linear.y = new_spot_velo[1,0]

    velo = []

    pub.publish(new_spot_velo[0,0], new_spot_velo[1,0], resume_checkpoint, current_checkpoint)
    
    past_time = current_time


def main():
    spot_kin = message_filters.Subscriber('/kinematic_state', spot_driver.msg.KinematicState)
    object_sub = message_filters.Subscriber('/zed2i/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped)

    ts = message_filters.ApproximateTimeSynchronizer([spot_kin, object_sub],queue_size = 100, slop = 0.1)
    ts.registerCallback(callback)

    # rospy.Subscriber('/zed2/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped, callback)
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("Subscriber_Node", anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass

