#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2, atan
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import matplotlib.pyplot as plt

class stanley_controller :
    def __init__(self):
        rospy.init_node('stanley_controller', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False
        self.current_postion=Point()    

        self.vehicle_length=4#필요없음
        self.k = 1.3  # gain값

        rate = rospy.Rate(0) # 30hz
        while not rospy.is_shutdown():
            if self.is_path ==True and self.is_odom==True  :
                
                vehicle_position=self.current_postion
                min_dist = float('inf')
                closest_idx = 0
                # closest point on the path
                closest_point = None
                for idx, pose in enumerate(self.path.poses):
                    dist = sqrt(pow(self.current_postion.x - pose.pose.position.x, 2) + pow(self.current_postion.y - pose.pose.position.y, 2))
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = idx
                        closest_point = pose.pose.position

                # Cross Track Error 계산
                cte = min_dist

                # determine the sign of cte
                vector_track = [self.path.poses[closest_idx+1].pose.position.x - closest_point.x, self.path.poses[closest_idx+1].pose.position.y - closest_point.y]
                vector_vehicle = [self.current_postion.x - closest_point.x, self.current_postion.y - closest_point.y]
                crosstrack_sign = np.sign(np.cross(vector_track, vector_vehicle))
                cte *= crosstrack_sign
                cte=-cte
                

                path_yaw = atan2(self.path.poses[closest_idx+1].pose.position.y - self.path.poses[closest_idx].pose.position.y, self.path.poses[closest_idx+1].pose.position.x - self.path.poses[closest_idx].pose.position.x)
                heading_error = path_yaw - self.vehicle_yaw
                if heading_error > pi:
                    heading_error -= 2 * pi
                if heading_error < -pi:
                    heading_error += 2 * pi

                # Stanley steering control
                self.ctrl_cmd_msg.steering = heading_error + atan(self.k * cte / (self.ctrl_cmd_msg.velocity + 0.1))
                self.ctrl_cmd_msg.velocity=30.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y




if __name__ == '__main__':
    try:
        test_track=stanley_controller()
    except rospy.ROSInterruptException:
        pass