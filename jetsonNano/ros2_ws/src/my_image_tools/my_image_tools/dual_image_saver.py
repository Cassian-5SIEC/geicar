import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime

class DualImageSaver(Node):
    def __init__(self):
        super().__init__('dual_image_saver')

        # 1. Declare Parameters (allows changing topics/path via command line)
        self.declare_parameter('image_topic_1', '/usb_cam_0/image_raw')
        self.declare_parameter('image_topic_2', '/usb_cam_1/image_raw')
        self.declare_parameter('save_directory', './captured_images')
        self.declare_parameter('image_encoding', 'bgr8')

        # 2. Retrieve Parameters
        self.topic_1 = self.get_parameter('image_topic_1').get_parameter_value().string_value
        self.topic_2 = self.get_parameter('image_topic_2').get_parameter_value().string_value
        self.save_dir = self.get_parameter('save_directory').get_parameter_value().string_value
        self.encoding = self.get_parameter('image_encoding').get_parameter_value().string_value

        # 3. Create Directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f'Created directory: {self.save_dir}')
        else:
            self.get_logger().info(f'Saving to existing directory: {self.save_dir}')

        # 4. Initialize CV Bridge
        self.bridge = CvBridge()

        # 5. Create Subscribers
        self.subscription_1 = self.create_subscription(
            Image,
            self.topic_1,
            self.listener_callback_1,
            10)
        
        self.subscription_2 = self.create_subscription(
            Image,
            self.topic_2,
            self.listener_callback_2,
            10)

        self.get_logger().info(f'Subscribed to: {self.topic_1} and {self.topic_2}')

    def listener_callback_1(self, msg):
        """Callback for the first camera topic."""
        self.process_and_save_image(msg, "cam1")

    def listener_callback_2(self, msg):
        """Callback for the second camera topic."""
        self.process_and_save_image(msg, "cam2")

    def process_and_save_image(self, msg, source_prefix):
        """
        Converts ROS Image message to OpenCV format and saves it to disk.
        """
        try:
            # Convert ROS Image message to OpenCV image
            # We use "bgr8" because OpenCV uses BGR by default
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Generate a unique filename using timestamp
        # Using msg.header.stamp is more accurate to the capture time, 
        # but using system time is easier for file sorting. 
        # Here we use system time for simplicity.
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{source_prefix}_{timestamp}.jpg"
        file_path = os.path.join(self.save_dir, filename)

        try:
            # Save the image
            success = cv2.imwrite(file_path, cv_image)
            if success:
                self.get_logger().info(f'Saved {filename}')
            else:
                self.get_logger().warn(f'Failed to save image at {file_path}')
        except Exception as e:
            self.get_logger().error(f'File IO Error: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = DualImageSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()