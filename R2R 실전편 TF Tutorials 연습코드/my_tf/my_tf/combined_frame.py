import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros


class CombinedTfBroadcaster(Node):
    def __init__(self):
        super().__init__('combined_tf_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.t = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def send_tf(self, parent, child, x, y, z, yaw):
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = parent
        t_msg.child_frame_id = child
        t_msg.transform.translation.x = x
        t_msg.transform.translation.y = y
        t_msg.transform.translation.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t_msg.transform.rotation.x = q[0]
        t_msg.transform.rotation.y = q[1]
        t_msg.transform.rotation.z = q[2]
        t_msg.transform.rotation.w = q[3]
        self.br.sendTransform(t_msg)

    def timer_callback(self):
        self.t += 0.05

        # world -> moving_frame: 반지름 2m 원, (0,0)을 바라보도록
        x1 = 2.0 * math.cos(self.t)
        y1 = 2.0 * math.sin(self.t)
        yaw1 = math.atan2(-y1, -x1)
        self.send_tf('world', 'moving_frame', x1, y1, 0.0, yaw1)

        # moving_frame -> child_frame: 반지름 1m 원, 2배 빠른 각도
        angle = 2.0 * self.t
        x2 = 1.0 * math.cos(angle)
        y2 = 1.0 * math.sin(angle)
        yaw2 = math.atan2(-y2, -x2)
        self.send_tf('moving_frame', 'child_frame', x2, y2, 0.0, yaw2)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
