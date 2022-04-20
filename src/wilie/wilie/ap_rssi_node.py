import rclpy
from rclpy.node import Node
from subprocess import Popen, PIPE
from std_msgs.msg import String

class NetworkReadingsNode(Node):
    def __init__(self):
        super().__init__('ap_rssi')
        self.readings_publisher_ = self.create_publisher(String, "WILIE2", 1)
        self.publisher_timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()

        # Get RSSI from WiFi devices
        with open('/proc/net/wireless', 'r') as file:
            data = file.read().split('\n')

        del data[0]
        del data[0]
        data.pop()

        metrics = ""

        try:
            for dev in data:
                metrics += (str(dev.split()[3]) + ' ')

            # Get throughput
            #process = Popen(['iperf3', '-c', '192.168.0.113', '-p', '5201', '-i', '0', '-u', '-t', '1'], stdout=PIPE, stderr=PIPE)
            #stdout, stderr = process.communicate()
            #metrics += stdout.decode('utf-8').split('\n')[3].split()[6]
            #self.get_logger().info(metrics)

        except:
            metrics = "oops"

        msg.data = metrics
        self.readings_publisher_.publish(msg)

    

        
def main(args=None):
    rclpy.init(args=args)
    node = NetworkReadingsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("banana")
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
