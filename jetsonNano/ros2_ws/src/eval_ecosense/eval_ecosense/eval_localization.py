# ROS 2 Imports
import math
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Python Import
import csv
import os
from datetime import datetime
import threading
import sys

class EvalLocalizationNode(Node):
    def __init__(self):
        super().__init__('eval_localization')

        # Mode
        self.mode = self.declare_parameter(
            'mode', 'absolute').get_parameter_value().string_value

        # TF Frames
        self.odom_frame = 'odom'
        self.base_link_frame = 'base_link'
        self.target_frame = self.declare_parameter(
            'target_frame', 'target').get_parameter_value().string_value
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a CSV to log data
        csv_name = f"eval_localization_{datetime.now().strftime('%Y_%m_%d-%H_%M_%S')}.csv"
        
        self.csv_path = self.declare_parameter(
            'csv_file_path', os.path.join(os.getcwd(), csv_name)
        ).get_parameter_value().string_value

        self.get_logger().info(f"[Eval] Logging localization data to: {self.csv_path}")

        try:
            self.csv_handle = open(self.csv_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_handle)

            # Write CSV Header
            self.csv_writer.writerow(['Timestamp_Start', 'Start_X', 'Start_Y', 'Start_Theta',
                                    'Timestamp_End', 'End_X', 'End_Y', 'End_Theta',
                                    'Real_X', 'Real_Y', 'Real_Theta'])
            self.csv_handle.flush()
        except IOError as e:
            self.get_logger().error(f"[Eval] Failed to open CSV file: {e}")
            self.csv_handle = None
            self.destroy_node()
            return

        self.test = threading.Thread(target=self.start_test, daemon=True)
        self.test.start()
        

    def get_transform(self, from_frame, to_frame):
        try:
            trans = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f"[Eval] Could not transform {to_frame} to {from_frame}: {ex}")
            return None
        return trans

    def get_yaw_from_transform(self,transform_msg):
        """
        Input: transform_msg (geometry_msgs/msg/Transform or TransformStamped)
        Output: yaw (float, in radians)
        """
        # Extract the quaternion
        # Use transform_msg.transform.rotation if it's a TransformStamped
        q = transform_msg.transform.rotation 
        
        # Formula for Yaw (z-axis rotation) from Quaternion
        # standard conversion: atan2(2(w*z + x*y), 1 - 2(y*y + z*z))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def log_data(self, start_transform, end_transform):
        if self.csv_handle: 
            self.csv_writer.writerow([
                str(start_transform.header.stamp.sec) + '.' + str(start_transform.header.stamp.nanosec),
                start_transform.transform.translation.x,
                start_transform.transform.translation.y,
                self.get_yaw_from_transform(start_transform),
                str(end_transform.header.stamp.sec) + '.' + str(end_transform.header.stamp.nanosec),
                end_transform.transform.translation.x,
                end_transform.transform.translation.y,
                self.get_yaw_from_transform(end_transform),
                input("Enter Real X position: "),
                input("Enter Real Y position: "),
                input("Enter Real Theta (in radians): ")
            ])
            self.csv_handle.flush()
            self.get_logger().info(f"[Eval] Logged data to CSV.")
        else:
            self.get_logger().error(f"[Eval] CSV file is not available for logging.")
        
        return

    def start_test(self):
        while rclpy.ok():
            self.get_logger().info(f"[Eval] Presse Enter to record start position...")
            input()
            start_tf = self.get_transform(self.odom_frame, self.base_link_frame)
            if not start_tf:
                self.get_logger().error("[Eval] Failed to get start transform.")
                return
            self.get_logger().info(f"[Eval] Presse Enter to record end position...")
            input()
            end_tf = self.get_transform(self.odom_frame, self.base_link_frame)
            if not end_tf:
                self.get_logger().error("[Eval] Failed to get end transform.")
                return
            self.log_data(start_tf, end_tf)



    def destroy_node(self):
        if self.csv_handle:
            self.csv_handle.close()
            self.get_logger().info(f"[Eval] Closed CSV file: {self.csv_path}")
        super().destroy_node()

def main(argv=None):
    rclpy.init()
    node = EvalLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()