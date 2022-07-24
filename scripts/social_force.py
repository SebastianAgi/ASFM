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

pub = rospy.Publisher('/obj_too_close', geometry_msgs.msg.Twist, queue_size=10)
warn_info = zed_interfaces.msg.FloatList()
velo = geometry_msgs.msg.Twist()
flag = False
avoid_left = False
avoid_right = False
close = False
x_vel = 0
y_vel = 0
resume_checkpoint = 0
human_id = None
crowd = []
crowd_data = np.empty((0,12), float)
spot_x = 0
spot_y = 0

past_time = 0
current_time = 0
acc = 1.6
v_des = 1.6
v_cur = 0



def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = np.linalg.norm(v1)
    v2_u = np.linalg.norm(v2)
    v1a = np.array([v1[0,0],v1[1,0]])
    v2a = np.array([v2[0,0],v2[1,0]])
    angle = np.arccos(np.dot(v1a,v2a)/(v1_u*v2_u))
    return angle

def leftNormalVector(vec):
    return np.array([[-vec[1,0]],
                     [vec[0,0]]])

def desired_force(desired_direction, vmax, current_velo, RelaxationTime):
    force = (desired_direction * vmax - current_velo) / RelaxationTime
    return force

#spot is an np array (x,y vector), ped is obj_det_.objects
def social_force(spot, ped):
    lambda_ = 2.0
    gamma = 0.35
    n = 2
    n_prime = 3
    force = 0
    
    for i in range(len(ped.objects)):

        diff = np.array([[ped.objects[i].position[0]-spot[0,0]], 
                         [ped.objects[i].position[1]-spot[1,0]]])

        diffDirection = unit_vector(diff)
        diffDirection = np.reshape(diffDirection, (2,1))

        velDiff = spot - np.array([[ped.objects[i].velocity[0]],
                                   [ped.objects[i].velocity[1]]])

        interactionVector = np.add(lambda_ * velDiff, diffDirection)
        interactionLength = np.linalg.norm(interactionVector)
        interactionDirection = interactionVector / interactionLength

        theta = angle_between(diffDirection, interactionDirection)
        
        B = gamma * interactionLength

        forceVelocityAmount = -math.exp(-np.linalg.norm(diff) / B - (n_prime * B * theta) * (n_prime * B * theta))

        forceAngleAmount = -np.sign(theta) * math.exp(-np.linalg.norm(diff) / B - (n * B * theta) * (n * B * theta))

        forceVelocity = forceVelocityAmount * interactionDirection

        forceAngle = forceAngleAmount * leftNormalVector(interactionDirection)

        force += forceVelocity + forceAngle

    return force

def move(delta_time,socialforce, desiredforce, current_velo):

    a = socialforce + desiredforce
    v = current_velo + delta_time * a

    return v


def callback(spot, obj_det):
    global past_time, current_time, velo, flag
    
    if flag == False:
    past_time = rospy.Time.now()
    flag = True
    
    RelaxationTime = 1
    v_max = 1.6
    goal_x = -4.922226052558839
    goal_y = -13.30729253667893
    spot_velo = np.array([[spot.velocity_of_body_in_odom.linear.x],
                          [spot.velocity_of_body_in_odom.linear.y]])

    # ********************************
    # need current spot velocity
    # need current coordinates of spot
    # need current coordinates of pedestrian x
    # need current orientation of pedestrian x
    # ********************************

    direction = np.array([[goal_x - spot.vision_tform_body.translation.x],
                          [goal_y - spot.vision_tform_body.translation.y]])
    desired_direction = unit_vector(direction)

    g_force = desired_force(desired_direction, v_max, spot_velo, RelaxationTime)
    s_force = social_force(spot_velo, obj_det)

    rospy.loginfo(g_force)
    rospy.loginfo(s_force)    

    current_time = rospy.Time.now()

    delta_time = float(rospy.Time.__str__(rospy.Time.__sub__(current_time,past_time)))/10**9
    
    new_spot_velo = move(delta_time,s_force, g_force, spot_velo)
    velo.linear.x = new_spot_velo[0,0]
    velo.linear.y = new_spot_velo[1,0]

    rospy.loginfo(velo)
    pub.publish(velo)
    
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
