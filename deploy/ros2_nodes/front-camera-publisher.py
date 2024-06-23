import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/go2/rgb/image_raw', 10)
        self.bridge = CvBridge()
        self.gstreamer_str = "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
        self.cap = cv2.VideoCapture(0)

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
