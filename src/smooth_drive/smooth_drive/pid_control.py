import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
# from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from gpiozero import Motor

class PID_Controller(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(String,'/micro_ros_arduino_node_publisher',self.listener_callback,10)
        self.subscription = self.create_subscription(Joy, '/joy', self.controller_callback, 5)
        self.subscription
        self.first_callback = True
        self.left_prev = 0
        self.right_prev = 0
        self.gear_reduction = 784.0 / 81.0
        self.wheel_circum = .0905 * np.pi
        # self.wheel_base = .201707-.01
        self.dist_const = self.wheel_circum / (512*4*self.gear_reduction)
        # self.x = 0
        # self.y = 0
        # self.theta = 0
        self.x_btn = 0
        self.cir_btn = 0
        self.turn_stick = 0

        self.motor_l = Motor(2,3)
        self.motor_r = Motor(9,10)
        self.dt = .1
        self.speedR = 0
        self.speedL = 0
        self.motor_speedR = 0
        self.motor_speedL = 0

        self.errorL_prev = 0
        self.errorR_prev = 0

        self.errorL_cum = 0
        self.errorR_cum = 0

        self.KP = .035
        self.KD = .005
        self.KI = .0025

        self.first_callback = True
        print("Smooth controller initialized")

    def ticks_velocity(self,ticks):
        return self.dist_const*ticks/self.dt

    def listener_callback(self,msg):
        data = msg.data.split(" ")
        new_left = int(data[0])
        new_right = int(data[1])
        if self.first_callback:
            self.first_callback = False
            self.left_prev = new_left
            self.right_prev = new_right
            return

        #porpotional 
        vR = self.ticks_velocity(new_right - self.right_prev)
        vL = self.ticks_velocity(new_left - self.left_prev)

        self.left_prev = new_left
        self.right_prev = new_right

        right_error = self.speedR - vR
        left_error = self.speedL - vL
        self.motor_speedR = max(min(1,self.motor_speedR+right_error*self.KP+self.errorR_prev*self.KD+self.errorR_cum*self.KI),0)
        self.motor_speedL = max(min(1,self.motor_speedL+left_error*self.KP+self.errorL_prev*self.KD+self.errorL_cum*self.KI),0)
        self.motor_r.forward(self.motor_speedR)
        self.motor_l.forward(self.motor_speedL)
        print("new left speed:",round(self.motor_speedL,2), "new right speed:", round(self.motor_speedR,2),"Vel L:",round(vL,3),"Vel R:",round(vR,3))

        self.errorR_prev = right_error
        self.errorL_prev = left_error
        self.errorR_cum += right_error
        self.errorL_cum += left_error


    def controller_callback(self,msg):
        '''this is to be used with sticky keys on the controller'''
        motor_constR = .3
        motor_constL = .3
        if msg.buttons[1] != self.x_btn:
            self.x_btn = not self.x_btn
        if msg.axes[2] != self.turn_stick:
            self.turn_stick = msg.axes[5]
        if msg.axes[2] < 0:
            motor_constL += abs(msg.axes[2])*.3
            motor_constR += msg.axes[2]*.3
        else:
            motor_constL += -1*(msg.axes[2])*.3
            motor_constR += (msg.axes[2])*.3

        if msg.buttons[2] != self.cir_btn:
            self.cir_btn = not self.cir_btn
            self.motor_speedR = 0
            self.motor_speedL = 0
        self.speedR = motor_constR * self.x_btn
        self.speedL = motor_constL * self.x_btn
        # print("left speed:",round(self.speedL,3),"right speed:",round(self.speedR,3))
        
def main(args=None):
    rclpy.init(args=args)
    pid_controller = PID_Controller()
    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()