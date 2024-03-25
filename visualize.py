import rospy
from mobility_control.msg import state
import math
import matplotlib.pyplot as plt
import numpy as np

vel_times = []
current_times = []
yaw_rate_times = []
yaw_rates = []
vels = []
references = np.array([[], []])
current = np.array([[], []])

class Visualize: #visualize 시간값 받기
    def __init__(self):
        rospy.init_node("visualize", anonymous = False)
        rospy.Subscriber("/vehicle_state", state, self.state_CB)
        rospy.Subscriber("/reference_value", state, self.reference_CB)

        self.current_time = 0.0
        self.current_yaw_rate = 0.0
        self.current_yaw = 0
        self.current_x=0
        self.current_y=0
        self.graph_time = 0.0
        self.reference_yaw_rate = 0.0
        self.msg = state()
        self.rate = rospy.Rate(100)


        self.reference_x=0
        self.reference_y=0
        self.reference_yaw=0
        self.reference_yaw_rate = 0
        self.reference_vel = 0

    def state_CB(self, msg):
        self.current_yaw_rate = msg.current_yaw_rate
        self.current_vel = msg.current_vel
        self.vel_record_time = msg.vel_record_time
        self.yaw_rate_record_time = msg.yaw_rate_record_time
        self.current_x=msg.current_x
        self.current_y=msg.current_y
        current[0] = np.append(current[0], self.current_x)
        current[1] = np.append(current[1], self.current_y)  # 이 부분 수정


        

        #self.graph_time = (self.record_time % 3600) % 60
        #rospy.loginfo("%f", self.graph_time)

        vel_times.append(self.vel_record_time)
        yaw_rate_times.append(self.yaw_rate_record_time)
        vels.append(self.current_vel)
        yaw_rates.append(self.current_yaw_rate)

        #times.append(self.graph_time)
        #rospy.loginfo(yaw_rates)
    
    def reference_CB(self, msg):

        self.current_time = msg.current_time

        self.reference_x=msg.reference_x
        self.reference_y=msg.reference_y
        self.reference_yaw=msg.reference_yaw
        self.reference_yaw_rate = msg.reference_yaw_rate
        self.reference_vel=msg.reference_vel
        references[0] = np.append(references[0], self.reference_x)
        references[1] = np.append(references[1], self.reference_y)  # 이 부분 수정

        #references.append(self.reference_yaw_rate)
        current_times.append(self.current_time)


    def draw(self):
        # self.vel_length = len(vel_times)
        # self.yaw_rate_length = len(yaw_rate_times)
        plt.subplot(1,2,1)

        plt.plot(references[0],references[1],  marker="o", color = "blue")
        plt.xlabel("x")
        plt.ylabel("y")
        #y_values = [self.reference_yaw_rate] * self.yaw_rate_length
        #plt.plot(y_values, color = 'red') reference 값을 표현
        plt.grid(True)
        #plt.clf()

        #plt.subplot(1,2,2)
        plt.plot(current[0], current[1], marker = "o", color = "red")
        #plt.xlabel("time[sec]")
        #plt.ylabel("reference yaw rate")
        # rospy.loginfo("%f", self.current_time)
        # plt.plot(times, vels, marker = "o")
        # #plt.ylim(-1.0, 1.0)
        # plt.xlabel("time[sec]")
        # plt.ylabel("current velocity")
        #plt.grid(True)

        plt.pause(0.001)
        plt.clf()


def main():
    visualize = Visualize()

    while not rospy.is_shutdown():
        visualize.draw()
        visualize.rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass