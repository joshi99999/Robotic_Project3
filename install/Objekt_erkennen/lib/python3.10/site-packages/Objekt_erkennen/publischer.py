import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.bridge = CvBridge()
        self.image = cv2.imread('katze_1.png')

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info('Image published')

def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
