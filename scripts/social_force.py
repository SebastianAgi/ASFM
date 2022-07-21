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

past_time = 0
current_time = 0
acc = 1.6
v_des = 1.6
v_cur = 0


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


def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def leftNormalVector(vec):
    return np.array([[-vec[0,1]],
                     [vec[0,0]]])

def desired_force(desired_direction, current_velo, RelaxationTime):
    force = (desired_direction * 1,6 - current_velo) / RelaxationTime
    return force

#spot is an np array (x,y vector), ped is obj_det_.objects
def social_force(spot, ped):
    lambda_ = 2.0
    gamma = 0.35
    n = 2
    n_prime = 3
    force = 0
    
    for i in range(len(ped.objects)):

        diff = np.array([[ped.objects[i].position[0]-spot[0]], 
                         [ped.objects[i].position[1]-spot[1]]])

        diffDirection = normalized(diff)

        velDiff = spot - np.array([[ped.objects[i].velocity[0]],
                                   [ped.objects[i].velocity[1]]])

        interactionVector = lambda_ * velDiff + diffDirection
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


def callback(velo_cam, obj_det, pose_cam):
    global past_time, current_time
    
    RelaxationTime = 1

    # ********************************
    # need current spot velocity
    # need current coordinates of spot
    # need current coordinates of pedestrian x
    # need current orientation of pedestrian x
    # ********************************

    g_force = desired_force(desired_direction, spot_velo, RelaxationTime)
    s_force = social_force(spot_velo, obj_det)

    current_time = rospy.Time.now() if past_time != 0 else 0.0001
    delta_time = current_time - past_time
    
    new_spot_velo = move(delta_time,s_force, g_force, spot_velo)
    
    past_time = current_time


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
