import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from gpiozero import Motor

class Motor_Subscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(Joy,'/joy',self.listener_callback,10)
        self.subscription
        self.logs = self.get_logger()
        self.motor_l = Motor(2,3)
        self.motor_r = Motor(9,10)
        self.motor_l_prev = 0
        self.motor_r_prev = 0
    
    def listener_callback(self, msg):
        motor_l_new = msg.axes[1]
        if motor_l_new != self.motor_l_prev:
            self.motor_l_prev = motor_l_new
            if motor_l_new < -.05:
                self.motor_l.backward(abs(motor_l_new)*.75)
                self.logs.info("Left Going Backwards: "+str(motor_l_new))
            elif motor_l_new > .05:
                self.motor_l.forward(motor_l_new*.75)
                self.logs.info("Left Going Forwards: "+str(motor_l_new))
            else:
                self.motor_l.stop()
                self.logs.info("Stopping left motor")
        motor_r_new = msg.axes[5]
        if motor_r_new != self.motor_r_prev:
            self.motor_r_prev = motor_r_new
            if motor_r_new < -.05:
                self.motor_r.backward(abs(motor_r_new)*.75)
                self.logs.info("Left Going Backwards: "+str(motor_r_new))
            elif motor_r_new > .05:
                self.motor_r.forward(motor_r_new*.75)
                self.logs.info("Left Going Forwards: "+str(motor_r_new))
            else:
                self.motor_r.stop()
                self.logs.info("Stopping left motor")

    def listener_callback2(self,msg):
        # this version uses sticky controller keys to set a desired speed
        pass


def main(args=None):
    rclpy.init(args=args)
    motor_subscriber = Motor_Subscriber()
    rclpy.spin(motor_subscriber)

    motor_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()