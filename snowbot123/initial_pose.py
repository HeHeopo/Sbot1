import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import math


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay', 3.0)

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        yaw = self.get_parameter('yaw').value
        delay = self.get_parameter('delay').value

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._pending = {'x': x, 'y': y, 'z': z, 'yaw': yaw}
        self.create_timer(0.5, self._check_and_publish)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f'Waiting for map frame, then publishing initial pose ({x}, {y}, {yaw})...')

    def _check_and_publish(self):
        if self._pending is None:
            return
        if not self._tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
            return

        p = self._pending
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = p['x']
        msg.pose.pose.position.y = p['y']
        msg.pose.pose.position.z = p['z']
        msg.pose.pose.orientation.z = math.sin(p['yaw'] / 2.0)
        msg.pose.pose.orientation.w = math.cos(p['yaw'] / 2.0)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published initial pose ({p["x"]}, {p["y"]}, {p["yaw"]})')
        self._pending = None


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
