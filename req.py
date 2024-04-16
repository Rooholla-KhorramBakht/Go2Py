import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Request, '/api/robot_state/request', 10)
        self.timer = self.create_timer(1, self.publish_message)

    def publish_message(self):
        msg = Request()
        # Populate the message fields
        msg.header.identity.id=80005
        msg.header.identity.api_id=1001
        msg.parameter = '{"name":"sport_mode","switch":0}'
        self.publisher_.publish(msg)
        self.get_logger().info('Published message')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
