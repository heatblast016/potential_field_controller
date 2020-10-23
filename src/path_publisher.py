#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan
import vectormath
import numpy as np
import math

def distance_to_target(goalx, goaly):
    global globalx, globaly, theta
    tx = (goalx-globalx)
    ty = (goaly-globaly)
    return math.sqrt(tx**2 + ty**2)

def setpose(m_x, m_y, m_theta):
    global globalx, globaly, theta
    globalx = m_x
    globaly = m_y
    theta = m_theta
def callbackpose(data):
    mx = data.pose.position.x
    my = data.pose.position.y
    explicit_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    angle = euler_from_quaternion(explicit_quat)
    mtheta = angle[2]
    setpose(mx, my, mtheta)
def VectorAdd(v1, v2):
    return(vectormath.vector.Vector2(v1.x + v2.x, v1.y + v2.y))
def run_plan(pub_init_pose, pub_controls, plan):
    global globalx, globaly, theta
    init = plan.pop(0)
    send_init_pose(pub_init_pose, init)

    for c in plan:
        cmd = c.split(",")
        assert len(cmd) == 2
        targ_x, targ_y = float(cmd[0]), float(cmd[1])
        print(c)
        while(distance_to_target(targ_x, targ_y) > 0.25):
            send_command(pub_controls, c)

def calculateSingle(target_x, target_y):
    global globalx, globaly, theta
    #returns overall target vector of car relative to map
    Vd = vectormath.vector.Vector2(target_x - globalx, target_y - globaly)
    VF = VectorAdd(Vd, VLaser)
    VF = VectorAdd(VF, VOther)
    VF = Vd
    return VF
def callbackscan(data):
    global VLaser
    VLaser =  vectormath.vector.Vector2(0, 0)
    angle = data.angle_min
    for i in data.ranges:
        VLaser = VectorAdd(VLaser, laserVector(i, angle))
        angle += data.angle_increment
    
    
def laserVector(dist, angle):
    if(math.isnan(dist)):
        dist = 100000000
    elif(dist == 0):
        dist = 100000000
    mag = kL/dist
    lv = vectormath.vector.Vector2(mag * math.cos(angle), mag * math.sin(angle))
    return lv

def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x1, y1, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x1, y=y1)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def callbackposeoc(data):
    global VOther
    mx = data.pose.position.x
    my = data.pose.position.y
    magnitude = math.sqrt((globalx-mx)**2+(globaly-my)**2)
    length = kL/magnitude
    konstant = length/magnitude
    VOther = vectormath.vector.Vector2(konstant*(globalx-mx), konstant*(globaly-my))

def send_command(pub_controls, c):
    global globalx, globaly, theta
    cmd = c.split(",")
    assert len(cmd) == 2
    target_x, target_y = float(cmd[0]), float(cmd[1])
    Vf = calculateSingle(target_x, target_y)
    vel = kP * math.sqrt((Vf.x ** 2) + (Vf.y ** 2))
    print("velocity", vel)
    vectorheading = (math.atan2(Vf.y, Vf.x))
    if(vectorheading-theta > math.pi):
        delta =  theta - ((2 * math.pi)- vectorheading)
    else:
        delta = vectorheading - theta
    

    dur = rospy.Duration(1.0)
    rate = rospy.Rate(10)
    start = rospy.Time.now()
    
    drive = AckermannDrive(steering_angle=delta * kH, speed=vel)

    while rospy.Time.now() - start < dur:
        pub_controls.publish(AckermannDriveStamped(drive=drive))
        rate.sleep()


if __name__ == "__main__":
# subscribe to particle filter
    globalx = 0.0 
    globaly = 0.0
    theta = 0
    kP = 0.4
    kH = 1
    kL = 10
    VLaser = None
    VOther = None
    rospy.init_node("path_publisher")
    sub_pf = rospy.Subscriber("/car1/car_pose" , PoseStamped, callbackpose, queue_size=10)
    sub_ls = rospy.Subscriber("/car1/scan", LaserScan, callbackscan)
    sub_oc = rospy.Subscriber("/car2/car_pose" , PoseStamped, callbackposeoc, queue_size=10)
    control_topic = rospy.get_param("~control_topic", "car1/mux/ackermann_cmd_mux/input/navigation")
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    plan_file = rospy.get_param("~plan_file")

    with open(plan_file) as f:
        plan = f.readlines()

    # Publishers sometimes need a warm-up time, you can also wait until there

    # are subscribers to start publishing see publisher documentation.
    rospy.sleep(1.0)
    run_plan(pub_init_pose, pub_controls, plan)

