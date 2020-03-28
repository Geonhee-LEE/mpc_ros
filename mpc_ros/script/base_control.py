#!/usr/bin/python
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import tf
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class BaseControl:
    def __init__(self):
        # Get params
        self.baseId = rospy.get_param('~base_id', 'base_footprint') # base link
        self.odomId = rospy.get_param('~odom_id', 'odom') # odom link
        self.devicePort = rospy.get_param('~port', '/dev/stm32base') # device port
        self.baudrate = float( rospy.get_param('~baudrate', '115200') ) # baudrate
        self.odom_freq = float( rospy.get_param('~odom_freq', '50') ) # hz of odom pub
        self.pwmRadian = float( rospy.get_param('~pwm_radian', '450') ) # non scaled factor pwm/radian
        self.neutralPt = float( rospy.get_param('~neutral_pt', '0') ) # steering neutral point (degree)
        self.carLength = float( rospy.get_param('~car_length', '0.1445') ) # unit: meter 
        self.wheelSep  = float( rospy.get_param('~wheel_separation', '0.156') ) # unit: meter 
        self.wheelRad  = float( rospy.get_param('~wheel_radius', '0.032') ) # unit: meter
        self.timeOut   = float( rospy.get_param('~time_out', '0.5') ) # unit: sec        
        self.VxCov = float( rospy.get_param('~vx_cov', '1.0') ) # covariance for Vx measurement
        self.VyawCov = float( rospy.get_param('~vyaw_cov', '1.0') ) # covariance for Vyaw measurement
        self.odom_topic = rospy.get_param('~odom_topic', '/odom') # topic name
        self.pub_tf = bool(rospy.get_param('~pub_tf', True)) # whether publishes TF or not
        self.debug_mode = bool(rospy.get_param('~debug_mode', False)) # true for detail info     
        self.steering_pwm_bound = float(rospy.get_param('~steering_pwm_bound', '240.0')) # unit: pwm
        self.throttle_pwm_bound = float(rospy.get_param('~throttle_pwm_bound', '128.0')) # unit: pwm, 93: 7.4v(~1.0 m/s), 131: 11.1v(~1.6 m/s)

        # Serial Communication
        try:
            self.serial = serial.Serial(self.devicePort, self.baudrate, timeout=10)
            rospy.loginfo("Flusing first 50 data readings ...")
            for x in range(0, 50):
                data = self.serial.read()
                time.sleep(0.01)

        except serial.serialutil.SerialException:
            rospy.logerr("Can not receive data from the port: "+ self.devicePort + 
            ". Did you specify the correct port ?")
            self.serial.close
            sys.exit(0) 
        rospy.loginfo("Communication success !")

        # ROS handler        
        self.sub_cmd  = rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, self.ackermannCmdCB, queue_size=10)
        self.sub_safe = rospy.Subscriber('ackermann_safe', AckermannDriveStamped, self.ackermannSafeCB, queue_size=10)
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)   
        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq), self.timerOdomCB) 
        self.timer_cmd = rospy.Timer(rospy.Duration(0.05), self.timerCmdCB) # 10Hz
        self.tf_broadcaster = tf.TransformBroadcaster() # TF

        # variable 
        self.neutralPt_radian = self.neutralPt*math.pi/180.0
        #self.steering_pwm_bound = 240.0 # unit: pwm
        #self.throttle_pwm_bound = 93.0
        self.cmd_steering_bound = self.steering_pwm_bound/self.pwmRadian # (pwm limit)/pwmRadian, unit: radian              
        self.cmd_steering = 0.0 # radian
        self.cmd_speed_bound = 2*math.pi*self.wheelRad*100.0/1560.0*self.throttle_pwm_bound # pwm to m/s formula (hardware spec), unit: m/s
        self.max_speed_compensate = self.cmd_speed_bound*self.wheelSep*math.tan(self.cmd_steering_bound - abs(self.neutralPt_radian))/(2.0*self.carLength) # find max rear wheel difference (m/s)
        self.compensate_factor = 0.75; # since the real compensation can only be found by iteration, we use a factor to approximate 
        self.cmd_speed = 0.0 # m/s
        self.cmd_time = rospy.Time.now()
        self.safe_steering = 0.0
        self.safe_speed = 0.0
        self.safe_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.previous_time = rospy.Time.now()
        self.timeOut_flag = 0
        self.pose_x = 0.0 # SI
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        # reading loop 
        while True:         
            reading = self.serial.read(6)
            if int(reading[0].encode('hex'),16) == 255 and int(reading[1].encode('hex'),16) == 254:
                self.data = reading
            else:
                self.serial.read(1)

    def ackermannCmdCB(self, data):
        self.cmd_steering = data.drive.steering_angle # radian
        if self.cmd_steering > (self.cmd_steering_bound - abs(self.neutralPt_radian)): # shrink the bound range by subtracting the neutral point
            self.cmd_steering = self.cmd_steering_bound - abs(self.neutralPt_radian)
        if self.cmd_steering < -(self.cmd_steering_bound - abs(self.neutralPt_radian)):
            self.cmd_steering = -(self.cmd_steering_bound - abs(self.neutralPt_radian))
        self.cmd_speed    = data.drive.speed
        if self.cmd_speed > (self.cmd_speed_bound - self.compensate_factor*self.max_speed_compensate): # decrease speed if compensation added value is higher than the bound 
            self.cmd_speed = self.cmd_speed_bound - self.compensate_factor*self.max_speed_compensate
        if self.cmd_speed < -(self.cmd_speed_bound- self.compensate_factor*self.max_speed_compensate):
            self.cmd_speed = -(self.cmd_speed_bound - self.compensate_factor*self.max_speed_compensate)
        self.cmd_time     = rospy.Time.now()
        self.timeOut_flag = 0 # reset

    # Relay: Safety commands
    def ackermannSafeCB(self, data):
        self.safe_steering = data.drive.steering_angle # radian
        if self.safe_steering > (self.cmd_steering_bound - abs(self.neutralPt_radian)): # shrink the bound range by subtracting the neutral point
            self.safe_steering = self.cmd_steering_bound - abs(self.neutralPt_radian)
        if self.safe_steering < -(self.cmd_steering_bound - abs(self.neutralPt_radian)):
            self.safe_steering = -(self.cmd_steering_bound - abs(self.neutralPt_radian))
        self.safe_speed    = data.drive.speed
        if self.safe_speed > (self.cmd_speed_bound - self.compensate_factor*self.max_speed_compensate): # decrease speed if compensation added value is higher than the bound 
            self.safe_speed = self.cmd_speed_bound - self.compensate_factor*self.max_speed_compensate
        if self.safe_speed < -(self.cmd_speed_bound- self.compensate_factor*self.max_speed_compensate):
            self.safe_speed = -(self.cmd_speed_bound - self.compensate_factor*self.max_speed_compensate)
        self.safe_time     = rospy.Time.now()
        self.timeOut_flag = 0 # reset
    
    def timerOdomCB(self, event):
        # Serial read & publish 
        try:           
            data = self.data            
            # Normal mode            
            if len(data) == 6:
                WL = -float( (int(data[2].encode('hex'),16)*256 + int(data[3].encode('hex'),16) -500)*100.0/1560.0*2*math.pi ) # unit: rad/sec
                WR = -float( (int(data[4].encode('hex'),16)*256 + int(data[5].encode('hex'),16) -500)*100.0/1560.0*2*math.pi )
            else:
                print 'Error Value! header1: ' + str(int(data[0].encode('hex'),16)) + ', header2: ' + str(int(data[1].encode('hex'),16))          

            # Twist
            VL = WL * self.wheelRad # V = omega * radius, unit: m/s
            VR = WR * self.wheelRad
            Vx = (VR+VL)/2.0
            Vyaw = Vx * math.tan(self.cmd_steering) / self.wheelSep;

            # Pose
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.previous_time).to_sec()
            self.previous_time = self.current_time
            self.pose_x   = self.pose_x   + Vx * math.cos(self.pose_yaw) * dt
            self.pose_y   = self.pose_y   + Vx * math.sin(self.pose_yaw) * dt
            self.pose_yaw = self.pose_yaw + Vyaw * dt
            pose_quat = tf.transformations.quaternion_from_euler(0,0,self.pose_yaw)
            
            # Publish Odometry
            msg = Odometry()
            msg.header.stamp = self.current_time
            msg.header.frame_id = self.odomId
            msg.child_frame_id  = self.baseId
            msg.pose.pose.position.x = self.pose_x
            msg.pose.pose.position.y = self.pose_y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.x =  pose_quat[0]
            msg.pose.pose.orientation.y =  pose_quat[1]
            msg.pose.pose.orientation.z =  pose_quat[2]
            msg.pose.pose.orientation.w =  pose_quat[3]
            msg.twist.twist.linear.x = Vx
            msg.twist.twist.angular.z = Vyaw
            for i in range(36):
                msg.twist.covariance[i] = 0
            msg.twist.covariance[0] = self.VxCov
            msg.twist.covariance[35] = self.VyawCov
            msg.pose.covariance = msg.twist.covariance
            self.pub.publish(msg)

            # TF Broadcaster
            if self.pub_tf:
                self.tf_broadcaster.sendTransform( (self.pose_x, self.pose_y, 0.0), pose_quat, self.current_time, self.baseId, self.odomId)          

            # Debug mode                      
            if self.debug_mode: 
                if len(data) == 6:
                    header_1 = int(data[0].encode('hex'),16)
                    header_2 = int(data[1].encode('hex'),16)
                    tx_1 = int(data[2].encode('hex'),16)
                    tx_2 = int(data[3].encode('hex'),16)
                    tx_3 = int(data[4].encode('hex'),16)
                    tx_4 = int(data[5].encode('hex'),16) 
                    rospy.loginfo("[Debug] header_1:%4d, header_2:%4d, tx_1:%4d, tx_2:%4d, tx_3:%4d, tx_4:%4d", header_1, header_2, tx_1, tx_2, tx_3, tx_4 )
            
        except: 
            #rospy.loginfo("Error in sensor value !") 
            pass            

    def timerCmdCB(self, event):
        # time out thresholding        
        if (rospy.Time.now() - self.cmd_time).to_sec() > self.timeOut and (rospy.Time.now() - self.safe_time).to_sec() > self.timeOut and self.timeOut_flag == 0:
            rospy.loginfo("Commands time out!")
            output = chr(255) + chr(254) + chr(0) + chr(0) + chr(0) + chr(0) + chr(0) #Vel_L, Vel_R, forward, steering, leftside 
            self.serial.write(output)
            self.timeOut_flag = 1
            return
        
        if self.timeOut_flag == 0:
            # check safety cmds
            cmd_steering = self.cmd_steering # local copy
            cmd_speed = self.cmd_speed
            if (rospy.Time.now() - self.safe_time).to_sec() <= self.timeOut:
                cmd_steering = self.safe_steering
                cmd_speed = self.safe_speed

            # compute steering cmd
            cmd_steering_adjust = cmd_steering + self.neutralPt_radian # add neutral point compensation (rad)
            steering_send = int(cmd_steering_adjust*self.pwmRadian) # convert pwm from radian
            leftside = 0 # 0: turn left, 1: turn right       
            if steering_send < 0:
                leftside = 1
                steering_send = -steering_send
            # steering limit
            if steering_send > int(self.steering_pwm_bound):
                steering_send = int(self.steering_pwm_bound)

            # compute speed cmd
            omega = cmd_speed/self.wheelRad # w = V/r*i (i is gear ratio), unit: rad/sec
            # calculate rear-wheel differential speed (ackermann effect)
            omega_left  = omega*(1 + self.wheelSep*math.tan(cmd_steering)/(2.0*self.carLength)) # w_L = omega(1+d/(2R)) R = L/tan(theta), unit: rad/s
            omega_right = omega*(1 - self.wheelSep*math.tan(cmd_steering)/(2.0*self.carLength)) # w_R = omega(1+d/(2R)) R = L/tan(theta), unit: rad/s  
            pwm_left  = abs(int(omega_left/100.0*1560.0/2.0/math.pi)) # rad/s to pwm formula (hardware spec)
            pwm_right = abs(int(omega_right/100.0*1560.0/2.0/math.pi))
            if pwm_left > int(self.throttle_pwm_bound): # [final check: shouldn't be called if everything is ok] if achieve max pwm, decrease both speed
                pwm_right = pwm_right - (pwm_left - int(self.throttle_pwm_bound))
                pwm_left = int(self.throttle_pwm_bound)
            if pwm_right > int(self.throttle_pwm_bound): # [final check: shouldn't be called if everything is ok] if achieve max pwm, decrease both speed
                pwm_left  = pwm_left - (pwm_right - int(self.throttle_pwm_bound))
                pwm_right = int(self.throttle_pwm_bound)

            forward = 0 # 0: forward, 1: reverse        
            if cmd_speed < 0.0:
                forward = 1

            #Protocal: Vel_L, Vel_R, forward, steering, leftside 
            output = chr(255) + chr(254) + chr(pwm_left) + chr(pwm_right) + chr(forward) + chr(steering_send) + chr(leftside)  
            #print 'left: ' + str(pwm_left) + ', right: ' + str(pwm_right)       
            self.serial.write(output)
        
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('base_control', anonymous=True)

        # Constract BaseControl Obj
        rospy.loginfo("HyphaROS MiniCar Base Control ...")
        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:    
        bc.serial.close        
        print("Shutting down")
