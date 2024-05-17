import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import math
import struct
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

# Constants
MIN_PULSE_WIDTH = 500 / 1_000_000  # For MG996R
MAX_PULSE_WIDTH = 2500 / 1_000_000  # For MG996R
SERVO_PIN = 12

class LidarToPointCloudNode(Node):

    def __init__(self):
        super().__init__('lidar_to_pointcloud_node')
        
        # Initialize servo
        factory = PiGPIOFactory()
        self.servo = Servo(SERVO_PIN, min_pulse_width=MIN_PULSE_WIDTH, max_pulse_width=MAX_PULSE_WIDTH, pin_factory=factory)

        # Initialize ROS2 subscribers and publishers
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10
        )
        self.point_cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)
        
        self.point_cloud = []
        self.scan_received = False
        self.scan_data = None

    def laser_scan_callback(self, msg):
        self.scan_data = msg
        self.scan_received = True

    def convert_scan_to_pointcloud(self, scan, vertical_angle):
        point_cloud = []
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        for i, distance in enumerate(scan.ranges):
            if scan.range_min < distance < scan.range_max:
                angle = angle_min + i * angle_increment
                x = distance * math.cos(angle) * math.cos(vertical_angle)
                y = distance * math.sin(angle) * math.cos(vertical_angle)
                z = distance * math.sin(vertical_angle)
                point_cloud.append([x, y, z])

        return point_cloud

    def publish_point_cloud(self):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        header = self.scan_data.header
        header.frame_id = "laser"

        point_cloud_data = np.array(self.point_cloud, dtype=np.float32).tobytes()
        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(self.point_cloud),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(self.point_cloud),
            data=point_cloud_data
        )

        self.point_cloud_pub.publish(point_cloud_msg)

    def start_scanning(self):
        try:
            while rclpy.ok():
                for vertical_angle_deg in range(-30, 31):
                    vertical_angle_rad = math.radians(vertical_angle_deg)
                    self.servo.value = math.sin(vertical_angle_rad)
                    sleep(0.1)

                    if self.scan_received:
                        self.scan_received = False
                        self.point_cloud.extend(self.convert_scan_to_pointcloud(self.scan_data, vertical_angle_rad))

                for vertical_angle_deg in range(30, -31, -1):
                    vertical_angle_rad = math.radians(vertical_angle_deg)
                    self.servo.value = math.sin(vertical_angle_rad)
                    sleep(0.1)

                    if self.scan_received:
                        self.scan_received = False
                        self.point_cloud.extend(self.convert_scan_to_pointcloud(self.scan_data, vertical_angle_rad))

                self.publish_point_cloud()
                self.point_cloud = []
        except KeyboardInterrupt:
            self.servo.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarToPointCloudNode()
    try:
        node.start_scanning()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
