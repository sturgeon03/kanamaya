#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mobility_control.msg import state
from math import pi, cos,sin


class reference_pub:
    def __init__(self):
        rospy.init_node("reference_pub", anonymous=False)
        self.pub = rospy.Publisher("/reference_value", state, queue_size=10) #시간값과 타겟값 보내기
        self.msg = state()
        self.frequency = 0.05
        self.time = rospy.get_time()
        self.rate = rospy.Rate(100)
        self.reference_yaw_rate = 0
        self.reference_vel = 0
        self.reference_x=0
        self.reference_y=0
        self.reference_yaw=0
        self.reference_yaw_rate=0
        

    def reference(self):
        #self.trans_time = rospy.Time.now()
        self.trans_time = rospy.get_rostime()
        self.current_time = self.trans_time.secs + self.trans_time.nsecs*1e-9
        #rospy.loginfo("%f", self.current_time)
        
        
        self.reference_yaw_rate = 0.1
        self.reference_yaw=self.reference_yaw_rate*self.current_time
        self.msg.current_time = self.current_time # 시간을 보내줌
        self.reference_vel = 10
        self.reference_x =self.reference_vel*cos(self.reference_yaw)*self.current_time
        self.reference_y =self.reference_vel*sin(self.reference_yaw)*self.current_time
        
        self.msg.reference_x= self.reference_x
        self.msg.reference_y=self.reference_y
        self.msg.reference_yaw=self.reference_yaw
        self.msg.reference_yaw_rate=self.reference_yaw_rate
        self.msg.reference_vel = self.reference_vel # 타겟 값을 보내줌
         #self.reference_yaw_rate = 0.1 * cos(2*pi*self.frequency*self.current_time) 
    def target_YawRate(self):
        #self.trans_time = rospy.Time.now()
        self.trans_time = rospy.get_rostime()
        self.current_time = self.trans_time.secs + self.trans_time.nsecs*1e-9
        #rospy.loginfo("%f", self.current_time)
        self.target_yaw_rate = 0.1 * cos(2*pi*self.frequency*self.current_time)
        self.msg.target_yaw_rate = self.target_yaw_rate # 타겟 값을 보내줌
        self.msg.current_time = self.current_time # 시간을 보내줌
    
   


def main():
    reference_pub_instance = reference_pub()  # 클래스 인스턴스 생성 시 변수 이름 변경
    while not rospy.is_shutdown():
        reference_pub_instance.reference()
        # rospy.loginfo(reference_pub_instance.msg)
        reference_pub_instance.pub.publish(reference_pub_instance.msg)
        reference_pub_instance.rate.sleep()
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 

        
