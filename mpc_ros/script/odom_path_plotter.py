#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy

import sys
import json
from math import sqrt
from collections import deque

import time

max_append = 0

def odomCallback(msg):
    global prev_odom_x, prev_odom_y, cnt

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
    pose = PoseStamped()    

    #Set a atributes of the msg
    pose.header.frame_id = "odom"
    pose.pose.position.x = float(msg.pose.pose.position.x)
    pose.pose.position.y = float(msg.pose.pose.position.y)
    pose.pose.orientation.x = float(msg.pose.pose.orientation.x)
    pose.pose.orientation.y = float(msg.pose.pose.orientation.y)
    pose.pose.orientation.z = float(msg.pose.pose.orientation.z)
    pose.pose.orientation.w = float(msg.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
    if (prev_odom_x != pose.pose.position.x and prev_odom_y != pose.pose.position.y):
            #Set a atributes of the msg
            pose.header.seq = path.header.seq + 1
            path.header.frame_id="odom"
            path.header.stamp=rospy.Time.now()
            pose.header.stamp = path.header.stamp
            #Published the msg
            path.poses.append(pose)

    cnt=cnt+1
    rospy.loginfo("Recorded count: %i" % cnt)
    if cnt > max_append:
        path.poses.pop(0)

    pub.publish(path)

    #Save the last position
    prev_odom_x=pose.pose.orientation.x; prev_odom_y=pose.pose.position.y
    return path


if __name__ == '__main__':
    #Variable initialization
    global prev_odom_x, prev_odom_y, cnt
    prev_odom_x=0.0; prev_odom_y=0.0; cnt=0
    #Node and msg initialization
    rospy.init_node('odom_path_plotter')

    #Rosparams that are set in the launch
    #max size of array pose msg from the path
    if not rospy.has_param("~max_list_append"):
        rospy.logwarn('The parameter max_list_append dont exists')
        max_append = rospy.get_param("~max_list_append", 10000)
    else: 
        max_append = 10000
    if not (max_append > 0):
        rospy.logwarn('The parameter max_list_append not is correct')
        sys.exit()
    pub = rospy.Publisher('/odom_path', Path, queue_size=1)
    path = Path() 
    msg = Odometry()

    #Subscription to the topic
    msg = rospy.Subscriber('/odom', Odometry, odomCallback) 
    rate = rospy.Rate(30) # 30hz

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass