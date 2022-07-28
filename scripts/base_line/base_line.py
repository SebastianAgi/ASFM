#!/usr/bin/env python3
import zed_interfaces.msg
import geometry_msgs.msg
import std_msgs.msg
import rospy
import math
import message_filters
import numpy as np
from numpy import *

roll = pitch = yaw = 0.0

pub = rospy.Publisher('/obj_too_close', zed_interfaces.msg.FloatList, queue_size=10)
warn_info = zed_interfaces.msg.FloatList()
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

def ellipse_center_dist(angle, a, b):
    x = a/(math.sqrt((a**2 * math.tan(angle)**2 / b**2) + 1))
    y = (a*math.tan(angle))/(math.sqrt((a**2 * math.tan(angle)**2 / b**2) + 1))
    dist = math.sqrt(x**2 + y**2)
    return dist, x, y

def distance(x,y):
    return math.sqrt(x**2 + y**2)

def get_minvalue(inputlist):
    #get the minimum value in the list
    min_value = min(inputlist)
    #return the index of minimum value 
    min_index=inputlist.index(min_value)
    return min_index

def ccw(A_x,A_y,B_x,B_y,C_x,C_y):
	return (C_y-A_y)*(B_x-A_x) > (B_y-A_y)*(C_x-A_x)

def intersect(A_x,A_y,B_x,B_y,C_x,C_y,D_x,D_y):
	return ccw(A_x,A_y,C_x,C_y,D_x,D_y) != ccw(B_x,B_y,C_x,C_y,D_x,D_y) and ccw(A_x,A_y,B_x,B_y,C_x,C_y) != ccw(A_x,A_y,B_x,B_y,D_x,D_y)

def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = dot( dap, db)
    num = dot( dap, dp )
    return (num / denom.astype(float))*db + b1

def quaternion_to_euler_angle_vectorized2(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z


def callback(velo_cam, obj_det, pose_cam):
    global flag, human_id, crowd_data, spot_x, spot_y, avoid_left, avoid_right, close, x_vel, y_vel, resume_checkpoint

    crowd_distance = []
    crowd = []
    avoid_left = False
    avoid_right = False
    close = False
    resume_checkpoint = 0
    spot_x = pose_cam.pose.position.x
    spot_y = pose_cam.pose.position.y

    #get rotation of camera around its Z-axis
    X, Y, Z = quaternion_to_euler_angle_vectorized2(pose_cam.pose.orientation.w, 
                                                    pose_cam.pose.orientation.x, 
                                                    pose_cam.pose.orientation.y, 
                                                    pose_cam.pose.orientation.z)

    spot_15m_x = spot_x + 30*np.cos(Z * np.pi/180)
    spot_15m_y = spot_y + 30*np.sin(Z * np.pi/180)

    for i in range(len(obj_det.objects)):

        #if human has already been detected before
        if obj_det.objects[i].label_id + 1 in crowd_data[:, 0]:         
            n,m = np.where(crowd_data == obj_det.objects[i].label_id + 1)
            crowd_data[n,4] = crowd_data[n,2]# new x_0
            crowd_data[n,5] = crowd_data[n,3]# new y_0
            crowd_data[n,2] = obj_det.objects[i].position[0]# new x
            crowd_data[n,3] = obj_det.objects[i].position[1]# new y
            
            if abs(obj_det.objects[i].velocity[0]) > 0.1:
                crowd_data[n,6] = obj_det.objects[i].velocity[0] # velo x
            else:
                crowd_data[n,6] = 0 
            if abs(obj_det.objects[i].velocity[1]) > 0.1:
                crowd_data[n,7] = obj_det.objects[i].velocity[1] # velo y
            else:
                crowd_data[n,7] = 0 

            crowd_data[n,8] = distance(crowd_data[n,6],crowd_data[n,7])
            if abs(crowd_data[n,3]) < 0.5 and abs(crowd_data[n,2] < 3):
                if crowd_data[n,3] < 0:
                    avoid_left = True
                else:
                    avoid_right = True
                if crowd_data[n,2] < 1:
                    close = True
                rospy.loginfo('avoiding still standing human')

            # rospy.loginfo('\n*********NEW CALLBACK*********\n'+str(len(obj_det.objects))+' human(s) detected' + '\nangle of cam: '+str(X) +str(Y) +str(Z) + '\ndist to line: ' + str(distance_to_line)+'\npoints: '+str(crowd_data[n,3]))
            
            #Checking if human is moving
            if crowd_data[n,8] != 0:
                crowd_data[n,9] = crowd_data[n,2] + ( (30/crowd_data[n,8]) * crowd_data[n,6]) #human final x
                crowd_data[n,10] = crowd_data[n,3] + ( (30/crowd_data[n,8]) * crowd_data[n,7])#human final y
                if intersect(pose_cam.pose.position.x,pose_cam.pose.position.y,30,pose_cam.pose.position.y,
                            crowd_data[n,2],crowd_data[n,3],crowd_data[n,9],crowd_data[n,10]):
                    A1 = array( [float(crowd_data[n,2]),float(crowd_data[n,3])] , dtype=float)   #current coords of human n
                    A2 = array( [float(crowd_data[n,9]),float(crowd_data[n,10])] , dtype=float)  #final coords of human n
                    B1 = array( [spot_x, spot_y] )         #current coords of spot/camera
                    B2 = array( [spot_15m_x, spot_15m_y] ) #final coords of spot/camera
                    intrsct_pnt = seg_intersect(A1,A2,B1,B2)
                    distan = distance(intrsct_pnt[0],intrsct_pnt[1])
                    if distan < 5:
                        strink = '\n{}{}\n{}{}\n{}\n'.format(pose_cam.pose.position.x,pose_cam.pose.position.y,crowd_data[n,2],crowd_data[n,3],distan)
                        # rospy.loginfo(strink)
                elif crowd_data[n,3] < 1.5:
                    nothing = 0
                    rospy.loginfo('avoid moving human!')


        #if human has not been detected before
        else:
            crowd.append([obj_det.objects[i].label_id + 1,                               #[0] ID
                          obj_det.header.stamp.secs+(obj_det.header.stamp.nsecs/10**9),  #[1] time stamp
                          obj_det.objects[i].position[0],                                #[2] x
                          obj_det.objects[i].position[1],                                #[3] y
                          obj_det.objects[i].position[0],                                #[4] x_0
                          obj_det.objects[i].position[1],                                #[5] y_0
                          0,                                                             #[6] velo x
                          0,                                                             #[7] velo y
                          0,                                                             #[8] |velo|
                          0,                                                             #[9] distance
                          0,                                                             #[10]
                          0])                                                            #[11]

    if crowd:
        crowd = np.asarray(crowd)
        if not np.any(crowd_data):
            crowd_data = crowd
        else:
            crowd_data = np.concatenate((crowd_data, crowd), axis=0)
    
    if close == True:
        if avoid_left == True:
            x_vel = 0
            y_vel = 0.5
        elif avoid_right == True:
            x_vel = 0
            y_vel = -0.5
    else:
        if avoid_left == True:
            x_vel = 0.5
            y_vel = 0.5
        elif avoid_right == True:
            x_vel = 0.5
            y_vel = -0.5

    if avoid_left == False and avoid_right == False:
        resume_checkpoint = 1

    warn_info = [0, 0, x_vel, y_vel, 0, resume_checkpoint]
    rospy.loginfo(warn_info)
    pub.publish(zed_interfaces.msg.FloatList(warn_info))
    
    

    # if flag == True:
    #     for i in range(len(obj_det.objects)):
    #         crowd_distance.append(obj_det.objects[i].label_id)
    #     if human_id not in crowd_distance:
    #         flag = False
    #         rospy.loginfo("leader no longer present, finding new leader")

    # if flag == False:
    #     if len(obj_det.objects) != 0:
    #         for i in range(len(obj_det.objects)):
    #             crowd_distance.append(distance(obj_det.objects[i].position[0], obj_det.objects[i].position[1]))        
    #         min_dist_human = get_minvalue(crowd_distance)
    #         human_id = obj_det.objects[min_dist_human].label_id
    #         flag = True
    #         rospy.loginfo("current closest human "+str(human_id)+" "+str(min(crowd_distance)))
     
    # for item in obj_det.objects:
        
    #     # rospy.loginfo(item.label_id)
    #     if (item.label_id != human_id):
    #         continue

    #     #Spot Ellipse major and minor axis:
    #     spot_el_a = 2.5
    #     spot_el_b = 2
    #     spot_forward_velocity = 1.5
    #     spot_backward_velocity = 2


    #     #Position of detected body
    #     current_human_pos = "\n(X,Y): ({},{})".format(round(item.position[0],3),round(item.position[1],3))

    #     current_human_dist = distance(item.position[0], item.position[1])

    #     #Angle of detected human relative to camera (zero degtrees in front of camera)
    #     angle = math.atan2(item.position[1],item.position[0])

    #     #Forbidden ellipse length infront of detected human
    #     length, x, y = ellipse_center_dist(angle, spot_el_a, spot_el_b)

    #     # Velocity to avoid human
    #     x_back = x - item.position[0]
    #     y_back = y - item.position[1]
    #     total_duration_back = length/spot_backward_velocity
    #     x_vel_b = x_back/total_duration_back
    #     y_vel_b = y_back/total_duration_back

    #     #velocity to follow human
    #     x_forward = item.position[0] - x
    #     y_forward = item.position[1] - y
    #     total_duration_forward = (current_human_dist-spot_el_a)/spot_forward_velocity
    #     x_vel_f = x_forward/total_duration_forward
    #     y_vel_f = y_forward/total_duration_forward
    #     angular = angle

    #     if current_human_dist <= length:
    #         warn_info = [length-current_human_dist, total_duration_back, -1*x_vel_b, -1*y_vel_b, 0]
    #         # rospy.loginfo("Human within safety area")
    #         # rospy.loginfo("move back {}".format(warn_info[0]))
    #         log_ = "***** Human %s *****"%item.label_id
    #         # rospy.loginfo(log_)
    #         # rospy.loginfo(warn_info[2])
    #         # rospy.loginfo(warn_info[3])
    #         pub.publish(zed_interfaces.msg.FloatList(warn_info))
    #     else:
    #         warn_info = [current_human_dist-spot_el_a, total_duration_forward, x_vel_f, y_vel_f, angle]
    #         # rospy.loginfo("Human within safety area")
    #         # rospy.loginfo("move back {}".format(warn_info[0]))
    #         log_ = "***** Human %s *****"%item.label_id
    #         # rospy.loginfo(log_)
    #         # rospy.loginfo(warn_info[0])
    #         # rospy.loginfo(warn_info[1])
    #         # rospy.loginfo(warn_info[2])
    #         # rospy.loginfo(warn_info[3])
    #         # rospy.loginfo(warn_info[4])
    #         pub.publish(zed_interfaces.msg.FloatList(warn_info))

    #     velo = "(vx,vy,vz): [{},{},{}]".format(round(item.velocity[0],2),round(item.velocity[1],2),round(item.velocity[2],2))
    
    # # rospy.loginfo(pose.pose)


def main():
    velo_cam = message_filters.Subscriber('/zed2i/zed_node/self_velocity', geometry_msgs.msg.TwistStamped)
    object_sub = message_filters.Subscriber('/zed2i/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped)
    pose_cam = message_filters.Subscriber('/zed2i/zed_node/pose', geometry_msgs.msg.PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer([velo_cam, object_sub, pose_cam],queue_size = 10, slop = 0.5)
    ts.registerCallback(callback)

    # rospy.Subscriber('/zed2/zed_node/obj_det/objects', zed_interfaces.msg.ObjectsStamped, callback)
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("Subscriber_Node", anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass

