from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import numpy as np
import signal

# global origin
# global indices
# global points
# global o_map
# global traj

points = []
# traj = []
indices = []
origin = [0,0]

class Grid_Marker_Sub(Node):
    def __init__(self):
            super().__init__('grid_test')
            # self.subscription = self.create_subscription(MarkerArray,'/trajectory_node_list',self.listener_callback,10)
            self.subscription = self.create_subscription(OccupancyGrid, "/map", self.listener_callback, 1)
            self.subscription
            self.traj_sub = self.create_subscription(MarkerArray, "/trajectory_node_list", self.traj_callback, 1)
            self.network_sub = self.create_subscription(String, "/WILIE2",self.network_callback, 1)
            self.cur_loc = 0

            # o_map = []
            # self.map = []
            # global origin

            # self.origin = [0,0]
            self.map_cpm = 1 / .05
            self.network_metric_file = open("/home/ubuntu/ros2_ws/network_metrics.txt","w")
            # self.network_indice_file = open("/home/ubuntu/ros2_ws/network_indice.txt","w")
            
            # self.network_map = open("grid.npy","wb")

    def convert_points_to_array(self,msg):
        list_of_points = []
        for pt in msg.markers[-2].points:
            list_of_points.append([pt.x,pt.y])
        for pt in msg.markers[-1].points:
            list_of_points.append([pt.x,pt.y])
        return list_of_points
        # if len(list_of_points) < 
    

    def network_callback(self,msg):
        global indices
        print("got network junk")
        self.network_metric_file.write(msg.data+"\n")
        indices.append(self.cur_loc)
        # self.network_indice_file.write(str(self.cur_loc)+"\n")


    def traj_callback(self, msg):
        global points
        self.cur_loc = len(msg.markers[-2].points)+len(msg.markers[-1].points) - 2
        points = self.convert_points_to_array(msg)
        # print(len(msg.markers),len(msg.markers[-1].points), msg.markers[-1])
        # self.cur_loc = [msg.markers[-1].points[-1].x,msg.markers[-1].points[-1].y]
        # print("updated location:", self.cur_loc)
        print(self.cur_loc,len(msg.markers[-3].points),len(msg.markers[-2].points),len(msg.markers[-1].points))

    def listener_callback(self, msg):
        global origin
        global o_map
        # print(len(msg.markers),len(msg.markers[-1].points), msg.markers[-1])
        # print("\n\n")
        print("got map")
        o_map = np.array(msg.data).reshape((msg.info.height,msg.info.width))
        if (origin != np.array([msg.info.origin.position.x,msg.info.origin.position.y])).any():
            print("change in origin")
            # print("old:",self.origin, "new:",np.array([msg.info.origin.position.x,msg.info.origin.position.y]),"shape:",self.map.shape)
            origin = np.array([msg.info.origin.position.x,msg.info.origin.position.y])
            self.map_cpm = 1 / msg.info.resolution
            


def shutdown_node(*args):
    print("shutting down node")
    rclpy.shutdown()
    with open("/home/ubuntu/ros2_ws/network_map.npy","wb") as f:
        np.save(f, o_map)
    with open("/home/ubuntu/ros2_ws/network_map_origin.txt",'w') as f:
        f.write(str(origin[0])+" "+str(origin[1]))
    with open("/home/ubuntu/ros2_ws/network_traj.txt",'w') as f:
        print(indices)
        print(len(points))
        for loc in indices:
            f.write(str(points[loc][0])+" "+str(points[loc][1])+"\n")
    return True


def main(args=None):
    rclpy.init(args=args)
    test = Grid_Marker_Sub()
    # rclpy.on_shutdown(shutdown_node)
    # while rclpy.ok():
    #     rclpy.spin_once(test)
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        shutdown_node()

    # test.destroy_node()
    # rclpy.shutdown


if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, shutdown_node)
        signal.signal(signal.SIGTERM, shutdown_node)
        main()
    except KeyboardInterrupt:
        shutdown_node()
    
