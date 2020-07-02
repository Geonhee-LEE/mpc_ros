#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from math import pow, atan2, sqrt
from math import sqrt, cos, pi, sin
import tf
from tf.transformations import quaternion_from_euler

odom_path = Path()
error_path = Path()
desired_path = Path()

robot_odom = Odometry()

odom_count = 0
sum_error = 0
error_x = 0

id = "odom"
trajectory_type = ""

current_path_x, current_path_y, current_path_theta = 0, 0, 0

def odom_cb(data):
    global odom_path
    global odom_count
    global robot_odom

    robot_odom = data
    odom_count = odom_count + 1

    if odom_count % 3 == 0:
        odom_path.header = data.header
        odom_path.header.frame_id = id

        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        pose.header.frame_id = id
        odom_path.poses.append(pose)

        odom_path_pub.publish(odom_path)
        generation_desired_path()

def generation_error_path():
    error_path.header.frame_id = id
    error_path.header.seq = 1

    pose = PoseStamped()
    pose.header.frame_id = id
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position.x = current_path_x
    pose.pose.position.y = current_path_y
    pose.pose.orientation.w = 1
    error_path.poses.append(pose)


    pose = PoseStamped()
    pose.header.frame_id = id
    error_path.header.seq = 2
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position.x = robot_odom.pose.pose.position.x
    pose.pose.position.y = current_path_y
    pose.pose.orientation.w = 1
    error_path.poses.append(pose)
    

    error_path_pub.publish(error_path)


def generation_desired_path():
    '''
    Create desired path
    '''
    global trajectory_type
    rospy.loginfo("generation_desired_path()")
    
    desired_path = Path()
    iter = 1000
    
    if trajectory_type == "circle":
        #circle        
        radius = 5
        period = 1000
        for t in range(0, iter):
            desired_path.header.stamp = rospy.get_rostime()
            desired_path.header.frame_id = id
            desired_path.header.seq = t

            pose = PoseStamped()
            pose.header.seq = t 
            pose.header.frame_id = id
            pose.header.stamp = rospy.get_rostime()
            pose.pose.position.x = radius * sin(2 * pi * t / period) #+ self.x_0
            pose.pose.position.y = -radius * cos(2 * pi * t / period) #+ self.y_0+
            grad =  atan2((-radius * cos(2 * pi * (t+1) / period)- pose.pose.position.y) , (radius * sin(2 * pi * (t + 1) / period)) - (pose.pose.position.x + 1e-5))
            q = quaternion_from_euler(0, 0, grad)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            desired_path.poses.append(pose)
        
    #epitrochoid
    if trajectory_type == "epitrochoid":
        R = 5
        r = 1
        d = 3
        period = 1000
        scale_factor = 1  
        for t in range(0, iter):
            desired_path.header.stamp = rospy.get_rostime()
            desired_path.header.frame_id = id
            desired_path.header.seq = t

            pose = PoseStamped()
            pose.header.seq = t
            pose.header.frame_id = id
            pose.header.stamp = rospy.get_rostime()
            pose.pose.position.x = scale_factor * ((R + r) * cos(2 * pi * t/ period) - d * cos(((R + r) / r) * 2 * pi * t / period))
            pose.pose.position.y = scale_factor * ((R + r) * sin(2 * pi * t/ period) - d * sin(((R + r) / r) * 2 * pi * t / period))
            grad =  atan2((5 * sin(2 * pi* (t+1) / period) * cos(2 * pi* (t+1) / period) / (sin(2 * pi * (t+1) / period) ** 2 + 1)- pose.pose.position.y) , (5 * cos(2 * pi* (t+1) / period) / (sin(2 * pi * (t+1) / period) ** 2 + 1)-pose.pose.position.x + 1e-5))
            q = quaternion_from_euler(0, 0, grad)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]        
            
            desired_path.poses.append(pose)
    
    #infinite
    if trajectory_type == "infinite":    
        period = 1000
        scale_factor = 1  
        for t in range(0, iter):
            desired_path.header.stamp = rospy.get_rostime()
            desired_path.header.frame_id = id
            desired_path.header.seq = t

            pose = PoseStamped()
            pose.header.seq = t
            pose.header.frame_id = id
            pose.header.stamp = rospy.get_rostime()
            pose.pose.position.x = 10 * cos(2 * pi* t / period) / (sin(2 * pi * t / period) ** 2 + 1)
            pose.pose.position.y = 10 * sin(2 * pi* t / period) * cos(2 * pi* t / period) / (sin(2 * pi * t / period) ** 2 + 1)
            grad =  atan2((10 * cos(2 * pi* (t+1) / period) / (sin(2 * pi * (t+1) / period) ** 2 + 1)- pose.pose.position.y)\
                        ,10 * sin(2 * pi* (t+1) / period) * cos(2 * pi* (t+1) / period) / (sin(2 * pi * (t+1) / period) ** 2 + 1)-pose.pose.position.x + 1e-5)
            q = quaternion_from_euler(0, 0, grad)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]      
            
            desired_path.poses.append(pose)
    
    #square
    if trajectory_type == "square":
        period = 1000
        l = 10
        PI = 3.141592
        x = 0.0
        y = 0.0
        for t in range(0, iter):
            desired_path.header.stamp = rospy.get_rostime()
            desired_path.header.frame_id = id
            desired_path.header.seq = t

            pose = PoseStamped()
            pose.header.seq = t
            pose.header.frame_id = id
            pose.header.stamp = rospy.get_rostime()

            if(t <= period * 0.25):
                x = 0
                y += l/(period*0.25)
                pose.pose.position.x = x
                pose.pose.position.y = y
                q = quaternion_from_euler(0, 0, PI/2)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            elif(t <= period * 0.5):
                x -= l/(period*0.25)
                pose.pose.position.x = x
                pose.pose.position.y = y
                q = quaternion_from_euler(0, 0, PI)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            elif(t <= period * 0.75):
                y -=  l/(period*0.25)
                pose.pose.position.x = x
                pose.pose.position.y = y
                q = quaternion_from_euler(0, 0, -PI/2)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            elif(t <= period):
                x += l/(period*0.25)
                pose.pose.position.x = x
                pose.pose.position.y = y
                q = quaternion_from_euler(0, 0, 0)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            desired_path.poses.append(pose)

    desired_path_pub.publish(desired_path) 
    desired_path = []


 
def calculate_error(path_x, path_y, path_theta, robot_x, robot_y, robot_theta):
    global error_x, sum_error
    error_x = abs(path_x - robot_x)
    sum_error = error_x + sum_error

    print("path_x: ", path_x)
    print("path_y: ", path_y)
    print("path_theta: ", path_theta)
    print("robot_x: ", robot_x)
    print("robot_y: ", robot_y)
    print("robot_theta: ", robot_theta)

def find_line_position(path_number, y):
    global current_path_x, current_path_y, current_path_theta, sum_error
    current_path_y = y

    if len(path_arr_yy) == path_number + 1:
        print("### Passing Last path ###")
        generateVel(0.0, 0.0)
        print("Total error sum :", sum_error)
        return

    start_pt_x, start_pt_y = desired_path.poses[path_number].pose.position.x, desired_path.poses[path_number].pose.position.y
    next_pt_x, next_pt_y = desired_path.poses[path_number+1].pose.position.x, desired_path.poses[path_number+1].pose.position.y 
        
    
    if desired_path.poses[path_number].pose.position.x == desired_path.poses[path_number+1].pose.position.x:
        current_path_x = desired_path.poses[path_number].pose.position.x
        current_path_theta = 1.570796
    elif desired_path.poses[path_number].pose.position.y == desired_path.poses[path_number+1].pose.position.y:
        current_path_y = desired_path.poses[path_number].pose.position.y
    else:
        current_path_x = (y - start_pt_y)*(next_pt_x - start_pt_x)/(next_pt_y - start_pt_y) + start_pt_x
        current_path_theta = atan2(next_pt_y - start_pt_y, next_pt_x - start_pt_x)



if __name__ == '__main__':
    rospy.init_node('path_node')
    rospy.loginfo("path_node is started!!")
    global trajectory_type
    trajectory_type =  rospy.get_param('~trajectory_type')
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)

    desired_path_pub = rospy.Publisher('/desired_path', Path, queue_size=500)
    odom_path_pub = rospy.Publisher('/recorded_path', Path, queue_size=10)
    error_path_pub = rospy.Publisher('/error_path', Path, queue_size=10)
    rospy.spin()
