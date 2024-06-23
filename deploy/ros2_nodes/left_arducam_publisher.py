import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from argparse import ArgumentParser

argpars = ArgumentParser()
argpars.add_argument('video_index', type=int,help="The video index of the camera", default=1)
argpars.add_argument('topic_name', type=int,help="The topic name corresponding to the given camera.", default=1)
args = argpars.parse_args()

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, args.topic_name, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(args.video_index)

    def publish_images(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                print('new frame')
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing image')
            else:
               self.get_logger().warn('Failed to capture image')
               break

        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        image_publisher.publish_images()
    except KeyboardInterrupt:
        pass

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
