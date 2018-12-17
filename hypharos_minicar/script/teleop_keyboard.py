#!/usr/bin/python

# This is a modified verison of turtlebot_teleop.py
# to fullfill the needs of HyphaROS MiniCar use case
# Copyright (c) 2018, HyphaROS Workshop
# 
# The original license info are as below:
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys, select, termios, tty, math
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

header_msg = """
Control HyphaROS Minicar!
-------------------------
Moving around:
        i     
   j    k    l
        ,     

w/x : increase/decrease throttle bounds by 10%
e/c : increase/decrease steering bounds by 10%
s   : safety mode
space key, k : force stop
anything else : keep previous commands

CTRL-C to quit
"""

# Func for getting keyboard value
def getKey(safety_mode):
    if safety_mode: # wait unit keyboard interrupt
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    else: # pass if not detected
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

# Func for showing current bounds 
def showInfo(speed_bound, angle_bound):
    return "current bounds:\tspeed %s\tangle %s " % (speed_bound, angle_bound)

# Main Func
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('minicar_teleop')
    pub_cmd = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=5)
    pub_safe = rospy.Publisher('/ackermann_safe', AckermannDriveStamped, queue_size=5)
    safe_mode = bool(rospy.get_param('~safety_mode', False)) # true for safety cmds 
    speed_i = float(rospy.get_param('~speed_incremental', 0.1)) # m/s
    angle_i = float(rospy.get_param('~angle_incremental', 5.0*math.pi/180.0)) # rad (=5 degree)
    speed_bound = float(rospy.get_param('~speed_bound', 2.0))
    angle_bound = float(rospy.get_param('~angle_bound', 30.0*math.pi/180.0))
    
    if safe_mode:
        print "Switched to Safety Mode !"

    moveBindings = {
            'i':(speed_i,0.0),
            'j':(0.0,angle_i),
            'l':(0.0,-angle_i),
            ',':(-speed_i,0.0),
               }

    boundBindings={
            'w':(1.1,1),
            'x':(.9,1),
            'e':(1,1.1),
            'c':(1,.9),
              }

    status = 0
    acc = 0.1
    target_speed = 0.0 # m/s
    target_angle = 0.0 # rad
    # Create AckermannDriveStamped msg object
    ackermann_msg = AckermannDriveStamped()
    #ackermann_msg.header.frame_id = 'car_id' # for future multi-cars applicaton 

    try:
        print(header_msg)
        print(showInfo(speed_bound, angle_bound))
        while(1):
            key = getKey(safe_mode)
            if key in moveBindings.keys():
                target_speed = target_speed + moveBindings[key][0]
                target_angle = target_angle + moveBindings[key][1]
            elif key in boundBindings.keys():
                speed_bound = speed_bound * boundBindings[key][0]
                angle_bound = angle_bound * boundBindings[key][1]
                print(showInfo(speed_bound, angle_bound))
                if (status == 14):
                    print(header_msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                target_speed = 0.0
                target_angle = 0.0
            elif key == 's' : # switch safety mode
                safe_mode = not safe_mode
                if safe_mode:
                    print "Switched to Safety Mode !"
                else:
                    print "Back to Standard Mode !"
            elif key == '\x03': # cltr + C
                break

            # Command constraints
            if target_speed > speed_bound:
                target_speed = speed_bound
            if target_speed < -speed_bound:
                target_speed = -speed_bound
            if target_angle > angle_bound:
                target_angle = angle_bound
            if target_angle < -angle_bound:
                target_angle = -angle_bound

            # Publishing command
            #ackermann_msg.header.stamp = rospy.Time.now() # for future multi-cars applicaton 
            ackermann_msg.drive.speed = target_speed
            ackermann_msg.drive.steering_angle = target_angle
            if safe_mode:
                pub_safe.publish(ackermann_msg)
            else:
                pub_cmd.publish(ackermann_msg)

    except Exception as e:
        print(e)

    finally:
        ackermann_msg.drive.speed = 0
        ackermann_msg.drive.steering_angle = 0
        pub_cmd.publish(ackermann_msg)
        pub_safe.publish(ackermann_msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


