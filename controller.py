
import rospy
import time
from mobility_control.msg import state
import numpy as np
from math import cos, sin, pi, sqrt


class Controller:
    def __init__(self):
        rospy.init_node("controller", anonymous=False)
        rospy.Subscriber("/vehicle_state", state, self.state_CB)
        rospy.Subscriber("reference_value", state, self.reference_CB)
        self.pub = rospy.Publisher("/control", state, queue_size=10)
        self.rate = rospy.Rate(50) # 50hz, 이는 loop의 속도를 제어하기 위해 사용됩니다.
        self.dt= 1.0/50.0 # 초당 횟수의 역수를 통해 dt를 올바르게 계산
        self.msg = state()

        #종방향 pid gain
        self.vel_Kp = 0.8
        self.vel_Ki = 0.01
        self.vel_Kd = 0.01
        
        #횡방향 pid gain
        self.Kp = 0.7
        self.Ki = 0.01
        self.Kd = 0.01
        
        self.prev_error = 0
        self.i_control = 0
        self.vel_i_control = 0


        #카나야마에서 계산된 목표값
        self.target_yaw_rate = 0
        self.target_vel = 0
        
        #current 값
        self.current_vel = 5.0
        self.current_x=0
        self.current_y=0
        self.current_yaw = 0.0  # 이 부분이 중요합니다.
        self.current_yaw_rate = 0

        #reference 값
        self.reference_x=0
        self.reference_y=0
        self.reference_yaw=0
        self.reference_yaw_rate = 0
        self.reference_vel = 0

    #현재 차량 상태 콜백
    def state_CB(self, msg):
        self.current_yaw =msg.current_yaw
        self.current_yaw_rate = msg.current_yaw_rate
        self.current_vel = msg.current_vel
        self.current_x= msg.current_x
        self.current_y
     #목표(참조) 차량상태 콜백   
    def reference_CB(self, msg):
        self.reference_x=msg.reference_x
        self.reference_y=msg.reference_y
        self.reference_yaw=msg.reference_yaw
        self.reference_yaw_rate = msg.reference_yaw_rate
        self.reference_vel =msg.reference_vel
        #rospy.loginfo("%f", self.target_yaw_rate)

    def kanayama(self):
        #종방향 거리gain값
        Kx=0.1
        #횡방향 거리gain값
        Ky=0.1
        #횡방향 각도gain값
        Kthata=0.1
        
        #current point
        p_current = np.array([self.current_x, self.current_y,self.current_yaw])
        #reference point
        p_reference = np.array([self.reference_x,self.reference_y,self.reference_yaw])

        theta=self.current_yaw
        #회전행렬을 통해 에러값 계산(아래의 경우로 나누는게 맞는지 모르겠음) 
        #reference 기준 좌측

        t = np.array([
                    [cos(theta), sin(theta), 0],
                    [-sin(theta),cos(theta), 0],
                    [0,         0,           1]])
        p_error = t @ (p_reference - p_current)
        
      

        #target velocity 구하기(참조값과 목표값은 다르거나 혹은 이를 pub해줘서 참조값을 목표값을 변경해줘야 하지 않을까 싶다)
        self.target_vel = self.reference_vel * cos(p_current[2])+Kx * p_error[0]
        
        #target yaw_rate 구하기(참조값과 목표값은 다르거나 혹은 이를 pub해줘서 참조값을 목표값을 변경해줘야 하지 않을까 싶다)
        self.target_yaw_rate = self.reference_yaw_rate + self.reference_vel*(Ky*(p_error[1])+Kthata*sin(p_error[2]))

            
    def steer_publish(self):
        self.yaw_rate_error = self.target_yaw_rate - self.current_yaw_rate
        self.p_control = self.yaw_rate_error * self.Kp
        self.i_control += self.Ki * self.yaw_rate_error * self.dt
        self.d_control = self.Kd * (self.yaw_rate_error - self.prev_error)/self.dt

        self.steer_angle = self.p_control + self.i_control + self.d_control + self.target_yaw_rate * 3.0
        self.prev_error = self.yaw_rate_error

        self.msg.steer_angle = self.steer_angle
        self.msg.yaw_rate_error = self.yaw_rate_error

    def accel_publish(self):
        self.vel_error = self.target_vel - self.current_vel
        self.vel_p_control = self.vel_error * self.vel_Kp
        self.vel_i_control += self.vel_Ki * self.vel_error * self.dt

        self.accel = self.vel_p_control + self.vel_i_control
        self.vel_prev_error = self.vel_error
        
        self.msg.accel = self.accel
        self.msg.vel_error = self.vel_error

def main():
    controller = Controller()

    while not rospy.is_shutdown():
        controller.kanayama()  # 인자 없이 메소드 호출
        controller.steer_publish()
        controller.accel_publish()
        controller.pub.publish(controller.msg)
        controller.rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass        


# # numpy 사용 예시
# self.target_vel = self.reference_vel * np.cos(p_error[2]) + Kx * p_error[0]

# # math 사용 예시
# self.target_vel = self.reference_vel * math.cos(p_current[2]) + Kx * p_error[0]
#np.cos(p_error[2]) 이와 같은 형식을 추천하는데 그 이유는 모르겠음