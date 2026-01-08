import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import socket
import struct
import json
import time

class InferenceClient:
    """Helper class to manage a single socket connection per camera."""
    def __init__(self, ip, port, logger, name="Generic"):
        self.ip = ip
        self.port = port
        self.logger = logger
        self.name = name
        self.sock = None
        self.connect()

    def connect(self):
        try:
            if self.sock: self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Optional: Set a timeout so it doesn't hang forever if server crashes
            self.sock.settimeout(5.0) 
            self.sock.connect((self.ip, self.port))
            self.logger.info(f"[{self.name}] Connected to GPU Server.")
        except Exception as e:
            self.logger.warn(f"[{self.name}] Connection failed: {e}")
            self.sock = None

    def infer(self, img):
        if self.sock is None:
            self.connect()
            if self.sock is None: return []

        try:
            # 1. Encode
            _, img_encoded = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            data = img_encoded.tobytes()

            # 2. Send
            self.sock.sendall(struct.pack('>I', len(data)) + data)

            # 3. Receive Length
            raw_len = self.recvall(4)
            if not raw_len: raise ConnectionError("Server closed")
            resp_len = struct.unpack('>I', raw_len)[0]

            # 4. Receive Data
            resp_data = self.recvall(resp_len)
            response = json.loads(resp_data.decode('utf-8'))
            return response.get('detections', [])

        except (BrokenPipeError, ConnectionResetError, ConnectionError, socket.timeout) as e:
            self.logger.warn(f"[{self.name}] Socket error: {e}. Reconnecting...")
            self.connect()
            return []
        except Exception as e:
            self.logger.error(f"[{self.name}] Inference error: {e}")
            return []

    def recvall(self, n):
        data = bytearray()
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet: return None
            data.extend(packet)
        return data


class EcoSenseNode(Node):
    def __init__(self):
        super().__init__('ecosense_detector')

        # --- PARAMETERS ---
        self.declare_parameter('server_ip', '127.0.0.1') 
        self.declare_parameter('server_port', 55001)
        
        server_ip = self.get_parameter('server_ip').value
        server_port = self.get_parameter('server_port').value

        self.bridge = CvBridge()

        # --- DUAL CLIENTS ---
        # We instantiate two separate clients. They act independently.
        self.client_left = InferenceClient(server_ip, server_port, self.get_logger(), "Left_Cam")
        self.client_right = InferenceClient(server_ip, server_port, self.get_logger(), "Right_Cam")

        # --- ROS SUBSCRIPTIONS ---
        self.sub_left = self.create_subscription(Image, '/usb_cam_left/image_raw', self.left_callback, 10)
        self.pub_left = self.create_publisher(Image, '/usb_cam_left/image_processed', 10)

        self.sub_right = self.create_subscription(Image, '/usb_cam_right/image_raw', self.right_callback, 10)
        self.pub_right = self.create_publisher(Image, '/usb_cam_right/image_processed', 10)

        # --- DETECTION PUBLISHERS ---
        self.pub_left_det = self.create_publisher(Detection2D, '/usb_cam_left/object_target', 10)
        self.pub_right_det = self.create_publisher(Detection2D, '/usb_cam_right/object_target', 10)

        self.get_logger().info(f"Node started. Target: {server_ip}:{server_port}")

    def left_callback(self, msg):
        # Pass the specific LEFT client
        self.process_image(msg, self.pub_left, self.pub_left_det, self.client_left)

    def right_callback(self, msg):
        # Pass the specific RIGHT client
        self.process_image(msg, self.pub_right, self.pub_right_det, self.client_right)

    def process_image(self, msg, publisher, det_publisher, client):
        """
        Generic processing function.
        It takes the 'client' object as an argument so it knows which socket to use.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Use the passed client (Left or Right) to infer
        detections = client.infer(cv_image)
        
        best_det = None
        max_area = -1.0

        if detections:
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                label = f"{det['class_name']} {det['score']:.2f}"
                color_ = (0,255,0)
                if (det['score'] < 0.8):
                    color_ = (0,0,255)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color_, 2)
                cv2.putText(cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_, 2)

                # Calculate area to find biggest box
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    best_det = det

            # If we found at least one detection, publish the biggest one
            if best_det is not None:
                x1, y1, x2, y2 = best_det['bbox']
                
                # Publish detection msg
                det_msg = Detection2D()
                det_msg.header = msg.header
                
                # Bounding Box
                # vision_msgs Pose2D has 'position' field which contains x, y
                det_msg.bbox.center.position.x = (x1 + x2) / 2.0
                det_msg.bbox.center.position.y = (y1 + y2) / 2.0
                det_msg.bbox.size_x = float(x2 - x1)
                det_msg.bbox.size_y = float(y2 - y1)
                
                # Hypotheses
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = best_det['class_name']
                hyp.hypothesis.score = float(best_det['score'])
                
                det_msg.results.append(hyp)
                
                det_publisher.publish(det_msg)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EcoSenseNode()
    
    # RECOMMENDED: Use MultiThreadedExecutor so callbacks can run in parallel
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()