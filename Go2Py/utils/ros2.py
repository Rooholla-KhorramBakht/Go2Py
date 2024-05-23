import struct
import threading
import time
import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
def ros2_init(args=None):
    rclpy.init(args=args)


def ros2_close():
    rclpy.shutdown()

class ROS2ExecutorManager:
    """A class to manage the ROS2 executor. It allows to add nodes and start the executor in a separate thread."""
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.nodes = []
        self.executor_thread = None

    def add_node(self, node: Node):
        """Add a new node to the executor."""
        self.nodes.append(node)
        self.executor.add_node(node)

    def _run_executor(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.terminate()

    def start(self):
        """Start spinning the nodes in a separate thread."""
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.start()

    def terminate(self):
        """Terminate all nodes and shutdown rclpy."""
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()
        if self.executor_thread:
            self.executor_thread.join()

class ROS2TFInterface(Node):

    def __init__(self, parent_name, child_name, node_name):
        super().__init__(f'{node_name}_tf2_listener')
        self.parent_name = parent_name
        self.child_name = child_name
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.T = None
        self.stamp = None
        self.running = True
        self.thread = threading.Thread(target=self.update_loop)
        self.thread.start()
        self.trans = None

    def update_loop(self):
        while self.running:
            try:
                self.trans = self.tfBuffer.lookup_transform(self.parent_name, self.child_name, rclpy.time.Time(), rclpy.time.Duration(seconds=0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
            time.sleep(0.01)    

    def get_pose(self):
        if self.trans is None:
            return None
        else:
            translation = [self.trans.transform.translation.x, self.trans.transform.translation.y, self.trans.transform.translation.z]
            rotation = [self.trans.transform.rotation.x, self.trans.transform.rotation.y, self.trans.transform.rotation.z, self.trans.transform.rotation.w]
            self.T = np.eye(4)
            self.T[0:3, 0:3] = R.from_quat(rotation).as_matrix()
            self.T[:3, 3] = translation
            self.stamp = self.trans.header.stamp.nanosec * 1e-9 + self.trans.header.stamp.sec
            return self.T

    def close(self):
        self.running = False
        self.thread.join()  
        self.destroy_node()