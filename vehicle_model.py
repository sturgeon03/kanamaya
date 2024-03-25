import rospy
import time
from mobility_control.msg import state
import math
from math import cos, sin, pi, sqrt

class Vehicle_model:
    def __init__(self):
        rospy.init_node("vehicle_model", anonymous=False)
        rospy.Subscriber("/control", state, self.state_CB)
        self.pub = rospy.Publisher("/vehicle_state", state, queue_size=10)

        self.car_length = 3.5
        self.current_yaw=0
        self.current_yaw_rate = 0
        self.current_x=0
        self.current_y=0
        self.current_vel = 0
        self.steer_angle = 0
        self.accel = 0
        self.rate = rospy.Rate(50)
        self.dt = 0.02


        self.msg = state()
        
    def state_CB(self, msg):
        self.steer_angle = msg.steer_angle
        self.accel = msg.accel

    def current_yawRate(self):
        self.current_yaw_rate = (self.current_vel * math.tan(self.steer_angle)) / self.car_length
        rospy.loginfo("%f", self.current_yaw_rate)

        self.msg.header.stamp = rospy.Time.now()
        self.yawRate_nsecs = self.msg.header.stamp.nsecs
        self.yawRate_secs = self.msg.header.stamp.secs 
        self.yaw_rate_record_time = self.yawRate_secs + self.yawRate_nsecs * 1e-9

        self.msg.yaw_rate_record_time = self.yaw_rate_record_time
        self.msg.current_yaw_rate = self.current_yaw_rate
        

    def cal_yaw(self):
        self.current_yaw += self.current_yaw_rate * self.dt

    def current_velocity(self):
        self.current_vel += self.accel * self.dt
        rospy.loginfo("%f", self.current_vel)

        self.msg.header.stamp = rospy.Time.now()
        self.vel_nsecs = self.msg.header.stamp.secs
        self.vel_secs = self.msg.header.stamp.nsecs * 1e-9
        self.vel_record_time = self.vel_secs + self.vel_nsecs

        self.msg.current_vel = self.current_vel
        self.msg.vel_record_time = self.vel_record_time
    
    
    def current_point(self):
        self.current_x +=self.current_vel*cos(self.current_yaw)*self.dt
        self.current_y +=self.current_vel*sin(self.current_yaw)*self.dt
        
        self.msg.current_x = self.current_x
        self.msg.current_y = self.current_y

def main():
    vehicle_model = Vehicle_model()

    while not rospy.is_shutdown():
        vehicle_model.current_yawRate()
        vehicle_model.current_velocity()
        vehicle_model.current_point()
        vehicle_model.pub.publish(vehicle_model.msg)
        vehicle_model.rate.sleep()

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        

 