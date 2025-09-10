#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticMapToOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('static_map_to_odom_broadcaster')

        # Lista dei robot da parametri
        self.declare_parameter('robots', ['t0', 't1'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value

        # Offset iniziale per ciascun robot (x, y) in metri
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

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Creazione delle trasformate statiche map->odom per tutti i robot
        static_transforms = []
        for robot in self.robot_names:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'{robot}_/odom'
            offset_x, offset_y = self.offsets.get(robot, (0.0, 0.0))
            t.transform.translation.x = offset_x
            t.transform.translation.y = offset_y
            t.transform.translation.z = 0.0
            # identità per rotazione
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            static_transforms.append(t)
            self.get_logger().info(f"Static transform map -> {robot}/odom ready.")

        # Pubblica tutte le trasformate statiche
        self.tf_broadcaster.sendTransform(static_transforms)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapToOdomBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
