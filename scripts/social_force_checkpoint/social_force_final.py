#!/usr/bin/env python3

from turtle import distance
import zed_interfaces.msg
import geometry_msgs.msg
import std_msgs.msg
import rospy
import time
import math
import os
import readchar
import sys
import message_filters
import numpy as np
from numpy import *
import spot_driver.msg

pub = rospy.Publisher('/obj_too_close', zed_interfaces.msg.FloatList, queue_size=10)
velo = zed_interfaces.msg.FloatList()
flag = False
follow_flag = False
people_nearby = False
leader_pos = np.array([[0],[0]])
leader_velo = np.array([[0],[0]])
resume_checkpoint = 1
past_time = 0
current_time = 0
current_checkpoint = 1
file_list = []
spot_velo = np.array([[0],[0]])
rot_vel = 0
rotation = 0
leader = None
leader_too_close = False
count = 0
alone_count = 0
distances = np.array([[0],[0]])
motion_count = 0

path = '/home/prl-orin/ros_ws/src' #Directory to load Pose file from
load_file_name = "Pose_load.txt"
completeName_load = os.path.join(path, load_file_name)
pose_file = open(completeName_load, "r")
for line in pose_file:
        stripped_line = line.strip()
        line_list = stripped_line.split()
        file_list.append(line_list)

#Pre-load pose coordinates
Poses = []
for i in range(9):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = float(file_list[3+i*12][1])
    pose.position.y = float(file_list[4+i*12][1])
    pose.position.z = float(file_list[5+i*12][1])
    pose.orientation.x = float(file_list[7+i*12][1])
    pose.orientation.y = float(file_list[8+i*12][1])
    pose.orientation.z = float(file_list[9+i*12][1])
    pose.orientation.w = float(file_list[10+i*12][1])
    Poses.append(pose)

def check_next_pose(x, y):
    global resume_checkpoint, Poses, current_checkpoint

    vec = np.array([[x - Poses[current_checkpoint].position.x],
                    [y - Poses[current_checkpoint].position.y]])

    if  np.linalg.norm(vec) < 1:
        current_checkpoint += 1
        if current_checkpoint == 3: # goal reached
            current_checkpoint = 2
            resume_checkpoint = 1

    return current_checkpoint, resume_checkpoint

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(vec1,vec2):

    temp_1 = vec1[0,0]*vec2[0,0] + vec1[1,0]*vec2[1,0]
    temp_2 = np.linalg.norm(vec1)*np.linalg.norm(vec2)
    ang_between = np.arccos(temp_1 / temp_2)

    return ang_between

def ellipse_center_dist(angle, a, b):

    x = a/(((a**2 * math.tan(angle)**2 / b**2) + 1)**0.5)
    y = (a*math.tan(angle))/(((a**2 * math.tan(angle)**2 / b**2) + 1)**0.5)
    dist = (x**2 + y**2)**0.5
    return dist, x, y

def a_b_from_velo(velo):
    tot = np.linalg.norm(velo)
    a = 1.5 + 2*tot
    b = 1 + tot/4
    return a, b

def c_rotation(vector, angle):
    rotated_vector = np.array([[np.cos(angle)*vector[0,0] - np.sin(angle)*vector[1,0]],
                               [np.sin(angle)*vector[0,0] + np.cos(angle)*vector[1,0]]])

    return rotated_vector

def cc_rotation(vector, angle):
    rotated_vector = np.array([[ np.cos(angle)*vector[0,0] + np.sin(angle)*vector[1,0]],
                               [-np.sin(angle)*vector[0,0] + np.cos(angle)*vector[1,0]]])

    return rotated_vector

def odom_to_baselink(spot, odom_dir):
    global rot_vel
    # spot rotation in global odom
    ang = math.atan2(2 * spot.rotation.z * spot.rotation.w + 2 * spot.rotation.x * spot.rotation.y, 1 - 2 * spot.rotation.y**2 - 2 * spot.rotation.z**2)

    # baselink desired direction to checkpoint in odometry
    bl_d_d = cc_rotation(odom_dir, ang)

    spot_dir = np.array([[1],
                         [0]])
    #angle between origin pose and goal point
    ang_between = angle_between(spot_dir,bl_d_d)
    # An inacurate angle to show determine if clockwise or counter clockwise rotation is needed 
    angle_to = np.arctan2(spot_dir[0,0]*bl_d_d[1,0] - bl_d_d[0,0]*spot_dir[1,0], spot_dir[0,0]*spot_dir[1,0] + bl_d_d[0,0]*bl_d_d[1,0])

    degree = ang_between*180/np.pi

    if follow_flag == False:
        if angle_to > 0:
            if degree > 45.0:
                rot_vel = 1.5
            elif int(degree) in range(10, 45):
                rot_vel = 0.5
            else:
                rot_vel = 0.05
        else:
            if degree > 45.0:
                rot_vel = -1.5
            elif int(degree) in range(10, 45):
                rot_vel = -0.5
            else:
                rot_vel = -0.05
    
    else:
        if angle_to > 0:
            if degree > 50.0:
                rot_vel = 1.0
            elif int(degree) in range(10, 50):
                rot_vel = 0.5
            else:
                rot_vel = 0.05
        else:
            if degree > 50.0:
                rot_vel = -1.0
            elif int(degree) in range(10, 50):
                rot_vel = -0.5
            else:
                rot_vel = -0.05

    # srtang = '\nrot_vel: {}\nangle_to:\n{}\nang_between:\n{}'.format(rot_vel,angle_to,ang_between)
    # rospy.loginfo(srtang)

    return bl_d_d, rot_vel, ang

def desired_force(desired_direction, vmax, current_velo, RelaxationTime):

    force = np.add(desired_direction*vmax, -current_velo) / RelaxationTime
    return force

#spot is an np array (x,y vector), ped is obj_det_.objects
def social_force(spot, ped):
    global spot_velo, s_force, follow_flag, people_nearby, leader_pos, distances, leader, leader_velo, leader_too_close, alone_count
    forces = np.array([[0],[0]])
    force = np.array([[0],[0]])
    threshold = 0.1
    lambda_ = 0.75
    ids = []
    dists = []
    to_or_away = []
    less_than_4 = []
    past_leader = []
    leader_too_close = False

    if follow_flag == False:
        A = 0.1 # 10 #5.5
        B = -2.2 # -10.2 #-2
        C = 6 # 10 #1.7
    else:
        A = 0.05
        B = -4
        C = 3


    A2 = 0.1
    B2 = -2.2
    C2 = 2.7

    # Creating an array with all current frame pedestrians' id,distance to spot, and if they are moving away or towards spot 
    for i in range(len(ped.objects)):
        #going towards or away from spot
        if ped.objects[i].label_id in distances:
            x,y = np.where(distances == ped.objects[i].label_id)
            new_dist = sqrt( ped.objects[i].position[0]**2 + ped.objects[i].position[1]**2 )
            if (new_dist - float(distances[int(x),1])) < -0.1:
                to_or_away.append(-1)   # coming closer is represented by (-1)
            else:
                to_or_away.append(0)
        else:
            to_or_away.append(0)

        #ID tags of people in current frame
        ids.append(ped.objects[i].label_id)
        #Distance to people in current frame
        dists.append(sqrt( ped.objects[i].position[0]**2 + ped.objects[i].position[1]**2 ))
        # Tag of people are less than 4 meters away
        if sqrt( ped.objects[i].position[0]**2 + ped.objects[i].position[1]**2 ) < 4.0:
            less_than_4.append(1)
        else:
            less_than_4.append(0)
        #crete empty column to populate if leader is no longer to be followed
        past_leader.append(0)

    distances = np.array([ids,dists,to_or_away, less_than_4, past_leader]).T
    rospy.loginfo(distances)

    # Check if leader is still present in current frame pedestrians
    if leader not in distances:
        leader = None
        rospy.loginfo('No leader')
    # If leader present but too far away, then don't count as leader anymore
    # or if leader is the only person within 4 m for more than 10 sec, then stop follow
    elif leader in distances:
        n,m = np.where(distances == leader)
        if distances[n,1] > 4.0:
            leader = None 
            rospy.loginfo('No leader')
        elif np.count_nonzero(distances[:,1] < 4.0) == 1:
            if alone_count == 75: # 1/15hz so 75 iter will be 5 sec
                leader = None
                distances[n,4] = 1
            alone_count += 1
        else:
            alone_count = 0
    
    for i in range(len(ped.objects)):

        # Get coordinates of where pedestrian in the current frame distances array 
        x, y = np.where(distances == ped.objects[i].label_id)

        # Pedestrian position realtive to camera/spot in local distance
        diff = np.array([[ped.objects[i].position[0] - 0.4], 
                         [ped.objects[i].position[1]]])
        diffDirection = unit_vector(diff)
        diffDirection = np.reshape(diffDirection, (2,1))

        ########################################
        # Circular model for static pedesrians #
        ########################################
        if ped.objects[i].action_state == 0:

            x = np.linalg.norm(diff)
            repulsive_force = A * math.exp(B*x + C) # Ae^(Bx + C)

            force = np.array([[diffDirection[0,0]*repulsive_force],
                              [diffDirection[1,0]*repulsive_force]])

            if ped.objects[i].label_id == leader:
                leader_pos = diff
                leader_velo = np.array([[0],[0]])
                force = np.array([[0],[0]])

                if np.linalg.norm(diff) < 1.5:
                    leader_too_close = True

        # If pedestrian is moving above threshold apply an elliptical force model
        else:

            ped_velo = np.array([[ped.objects[i].velocity[0]],
                                 [ped.objects[i].velocity[1]]])

            if ped.objects[i].label_id == leader:
                leader_pos = diff
                leader_velo = ped_velo
                force = np.array([[0],[0]])

                if np.linalg.norm(diff) < 0.5:
                    leader_too_close = True

            ped_spot_ang = angle_between(unit_vector(ped_velo),unit_vector(np.array([[1],[0]])))

            x_ind,y_ind = np.unravel_index(np.argmin(distances, axis = None),distances.shape)

            if ped_spot_ang*180/np.pi < 30 and distances[x_ind,0] == ped.objects[i].label_id and distances[x_ind,1] < 4.0 and leader == None and people_nearby == True and distances[x_ind,4] != 1:
                strong = '\nthere is a leader now, id:{}\nangle:{}'.format(ped.objects[i].label_id,ped_spot_ang*180/np.pi)
                
                leader = ped.objects[i].label_id
                follow_flag = True
                leader_pos = diff
                leader_velo = ped_velo
                force = np.array([[0],[0]])
                if np.linalg.norm(diff) < 1.5:
                    leader_too_close = True

            else:

                #############################################
                # Force only if pedestrian is coming closer # 
                #############################################
                if distances[x,2] != -1:
                    force = np.array([[0],[0]])

                ####################
                # Elliptical model #
                ####################
                else:
                    a,b = a_b_from_velo(ped_velo)

                    ell_dis, x, y = ellipse_center_dist(ped_spot_ang, a, b)

                    repulsive_force = A * math.exp((x-ell_dis)*B + C)

                    force = np.array([[diffDirection[0,0]*repulsive_force],
                                      [diffDirection[1,0]*repulsive_force]])

        forces = np.add(forces,force)
        # rospy.loginfo(forces)
        if np.linalg.norm(forces) > 0:
            people_nearby = True
        else:
            people_nearby = False

    return forces

def move(delta_time,socialforce, desiredforce, current_velo, allowed_v, follow_flag):
    global leader_velo, count, distances
    multiplier = 0

    # if follow_flag == True:
    #     a = 10*desiredforce
    # else:

    # reinforce sideways walk if directed head on
    # rospy.loginfo('\nsf(0,0)'+str(socialforce[0,0])+'\nsc(1,0):'+str(socialforce[1,0]))
    # if socialforce[0,0] == 0 and socialforce[1,0] == 0:
    #     ratio = 1.0
    # else:
    #     ratio = socialforce[0,0]/abs(socialforce[1,0])

    # if ratio > 1.5:
    #     socialforce[1,0] = ratio*socialforce[1,0]

    a = np.add(-socialforce,desiredforce)

    v = current_velo + delta_time * a
    
    max_velo = allowed_v
    # rospy.loginfo('\nallowed_velo: '+str(allowed_v))
    
    if leader != None:
        x,y = np.where(distances == leader)
        if distances[int(x),1] < 1.75:
            max_velo = np.linalg.norm(leader_velo)

    length = sqrt(v[0,0]**2 + v[1,0]**2)

    if length > max_velo and max_velo != 0:
        multiplier = max_velo/length
    else:
        multiplier = 1
    
    clampedv = np.array([[v[0,0]*multiplier],[v[1,0]*multiplier]])
    

    # stop backwards walking, but allow it after waitibg for (75/15) = 5 sec
    # waiting to allow backward might not work as oncoming people can give negative 
    # x velo if the walk for long enough towads spot

    ###############################################################################
    # Either do this or do not allow backward walking, but have spot sit and wait #
    ###############################################################################
    # actually might be good to sit after having made some distance -\('_')/-
    if clampedv[0,0] < -0.01:
        count += 1
        if count > 75:
            rospy.loginfo('wait over')
        else:
            clampedv[0,0] = 0
    if clampedv[0,0] > 0.0:
        count = 0


    if leader_too_close == True:
        clampedv = np.zeros([2,1])

    # rospy.loginfo('\nspot_velo:' + str(clampedv))
    return clampedv

def duration(spot_pose,ped, cc):
    global Poses, distances
    velocity = 1
    ped_dist = np.empty((0), float)

    # for i in range(len(ped.objects)):
    #     temp = np.linalg.norm(np.array([[ped.objects[i].position[0]],[ped.objects[i].position[1]]]))
    #     temp_arr = np.array([temp])
    #     ped_dist = np.append(ped_dist, temp_arr, axis=0)

    if len(ped.objects) != 0:
        if distances[:,1].min() > 3 and distances[:,1].min() < 4:
            velocity = 1.0
        elif distances[:,1].min() < 3:
            velocity = 0.7
        else:
            velocity = 1.6
    
    distance_to_checkpoint = np.linalg.norm(np.array([[Poses[cc].position.x - spot_pose[0,0]],
                                                      [Poses[cc].position.y - spot_pose[1,0]]]))

    dur = distance_to_checkpoint/velocity

    return dur, velocity
        

def callback(spot, obj_det):
    global past_time, current_time, spot_velo, velo, flag, current_checkpoint, resume_checkpoint, Poses, rot_vel
    global follow_flag, leader_pos, rotation, leader, motion_count
    allowed_velo = 1
    angle_to_goal = 0
    RelaxationTime = 0.5
    v_max = 1

    real_spot_velo = np.array([[spot.velocity_of_body_in_vision.linear.x],
                               [spot.velocity_of_body_in_vision.linear.y]])

    if np.linalg.norm(real_spot_velo) < 0.1 and np.linalg.norm(spot_velo) > 0.1:
        motion_count +=1
    else:
        motion_count = 0
            

    if flag == False:
        past_time = rospy.Time.now()
        spot_velo = real_spot_velo
        flag = True

    s_force = social_force(spot_velo, obj_det)

    spot_position = np.array([[spot.vision_tform_body.translation.x],
                              [spot.vision_tform_body.translation.y]])

    if leader == None:
        follow_flag = False

    if follow_flag == True:
        odom_leader_pos = c_rotation(leader_pos,rotation)
        goal_x = spot_position[0,0] + odom_leader_pos[0,0]
        goal_y = spot_position[1,0] + odom_leader_pos[1,0]

        stringu = '\ngoal_x: {}\ngoal_y :{}\nspot_coord: {}\n'.format(goal_x,goal_y, spot_position)
        # rospy.loginfo(stringu)
    
    else:
        goal_x = Poses[current_checkpoint].position.x
        goal_y = Poses[current_checkpoint].position.y

    direction = np.array([[goal_x - spot_position[0,0]],
                          [goal_y - spot_position[1,0]]])

    odom_desired_direction = unit_vector(direction)

    desired_direction, rot_vel, rotation = odom_to_baselink(spot.vision_tform_body, odom_desired_direction)
    
    g_force = desired_force(desired_direction, v_max, spot_velo, RelaxationTime)
    
    current_checkpoint, resume_checkpoint = check_next_pose(spot_position[0,0],spot_position[1,0])

    # if np.linalg.norm(s_force) > threshold:
    #     resume_checkpoint = 0
    # else:
    #     resume_checkpoint = 1

    time, allowed_velo = duration(spot_position, obj_det, current_checkpoint)

    current_time = rospy.Time.now()
    delta_time = float(rospy.Time.__str__(rospy.Time.__sub__(current_time,past_time)))/10**9
    past_time = current_time
    
    new_spot_velo = move(delta_time,s_force, g_force, spot_velo, allowed_velo, follow_flag)

    if motion_count == 45:
        rospy.loginfo('I should wait now')
        new_spot_velo = np.zeros([2,1])
        #put in entry in FloatList to make spot sit/oor shake or give some sort of signal

    velo = [new_spot_velo[0,0], new_spot_velo[1,0], 0, current_checkpoint+1, time, rot_vel]
    spot_velo = new_spot_velo

    # if len(obj_det.objects) != 0:
    #     forces = '\nrepulsive:  {}    attractive: {}          velo: {}\n           {}               {}               {}\n\nrepul norm: {},    attra norm: {},     velo norm: {}\nvelo: {}\ndt: {}'.format(round(-s_force[0,0],3),
    #                                                                                                                                                                                     round(g_force[0,0],3),
    #                                                                                                                                                                                     round(new_spot_velo[0,0],3),
    #                                                                                                                                                                                     round(-s_force[1,0],3),
    #                                                                                                                                                                                     round(g_force[1,0],3),
    #                                                                                                                                                                                     round(new_spot_velo[1,0],3),
    #                                                                                                                                                                                     round(np.linalg.norm(s_force),3),
    #                                                                                                                                                                                     round(np.linalg.norm(g_force),3),
    #                                                                                                                                                                                     round(np.linalg.norm(new_spot_velo)),
    #                                                                                                                                                                                     velo,delta_time)
    #     rospy.loginfo(forces)

    # strink = '\n{}\n{}'.format(np.linalg.norm(s_force),velo)
    # rospy.loginfo(velo)

    pub.publish(velo)
    


def main():
    spot_kin = message_filters.Subscriber('/kinematic_state', spot_driver.msg.KinematicState)
    object_sub = message_filters.Subscriber('/zed2i/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped)

    ts = message_filters.ApproximateTimeSynchronizer([spot_kin, object_sub],queue_size = 100, slop = 0.08)
    ts.registerCallback(callback)

    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("Subscriber_Node", anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
