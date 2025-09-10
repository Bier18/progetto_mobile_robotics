#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class GlobalPosePublisher(Node):
    def __init__(self):
        super().__init__('global_pose_publisher')

        # Lista dei robot da parametri
        self.declare_parameter('robots', ['t0', 't1'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value

        # Offset iniziale di ogni robot nel frame globale (map)
        self.offsets = {
            't0': (-5.0, 0.0),
            't1': (-4.0, 0.0),
            't2': (-3.0, 0.0),
            't3': (-2.0, 0.0),
            't4': (-1.0, 0.0),
            't5': (0.0, 0.0),
            't6': (1.0, 0.0),
            't7': (2.0, 0.0),
            't8': (3.0, 0.0),
            't9': (4.0, 0.0),
            't10': (5.0, 0.0),
            't11': (6.0, 0.0),
            't12': (7.0, 0.0)
        }

        # Publisher PoseStamped per ogni robot
        self.pose_publishers = {}
        for robot in self.robot_names:
            topic = f'/{robot}/global_pose'
            self.pose_publishers[robot] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Publishing global pose on {topic}")

        # Subscriber su /<robot>/odom per ogni robot
        for robot in self.robot_names:
            topic = f'/{robot}/odom'
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def odom_callback(self, msg: Odometry, robot: str):
        now = self.get_clock().now().to_msg()

        # Lettura odometria (odom -> base_footprint)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation

        # Offset iniziale (map -> odom)
        offset_x, offset_y = self.offsets.get(robot, (0.0, 0.0))

        # Calcolo della posizione globale: map -> base_footprint
        global_x = x + offset_x
        global_y = y + offset_y
        global_z = z

        # Creazione PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = global_x
        pose_msg.pose.position.y = global_y
        pose_msg.pose.position.z = global_z
        pose_msg.pose.orientation = q

        # Pubblica sul topic custom
        self.pose_publishers[robot].publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
