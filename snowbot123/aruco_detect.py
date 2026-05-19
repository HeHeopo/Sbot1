import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros


class ArucoFollowNode(Node):
    def __init__(self):
        super().__init__('aruco_follow')

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('spin_speed', 0.5)
        self.declare_parameter('total_markers', 4)
        self.declare_parameter('enabled', True)
        self.declare_parameter('parent_frame', 'camera_link')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter(
            'camera_matrix_override',
            [381.0, 0.0, 320.0, 0.0, 381.0, 240.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter('camera_matrix_use_override', False)

        dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.total_markers = self.get_parameter('total_markers').value
        self.enabled = self.get_parameter('enabled').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.goal_frame = self.get_parameter('goal_frame').value

        dict_attr = getattr(cv2.aruco, dict_name, None)
        if dict_attr is None:
            self.get_logger().error(f'Unknown ArUco dictionary: {dict_name}')
            raise RuntimeError(f'Unknown dictionary: {dict_name}')
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
        self.bridge = CvBridge()

        override = self.get_parameter('camera_matrix_override').value
        use_override = self.get_parameter('camera_matrix_use_override').value
        if use_override:
            self.camera_matrix = np.array(override, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
            self.get_logger().info(
                f'Using override camera matrix:\n{self.camera_matrix}'
            )
        else:
            self.camera_matrix = None
            self.dist_coeffs = None
            self._calibration_timeout = self.create_timer(
                3.0, self._calibration_fallback
            )

        self.visited_markers = set()
        self.marker_positions = {}
        self.state = 'SEARCHING'
        self.wait_timer = None
        self._target_marker_id = None
        self.last_seen_time = self.get_clock().now()

        self._goal_handle = None
        self._nav_start_time = None
        self._nav_goal_sent = False

        self.R_cv_to_ros = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])

        callback_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, 10, callback_group=callback_group
        )

        self.control_timer = self.create_timer(0.1, self.control_callback)

        if self.enabled:
            self.create_subscription(
                Image, self.get_parameter('image_topic').value,
                self.image_callback, 10
            )
            self.get_logger().info('Image subscription started')

        self.get_logger().info(
            f'ArUco Nav2 started. Markers: {self.total_markers}, '
            f'spin: {self.spin_speed} rad/s'
        )

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(
                f'Camera calibration received:\n{self.camera_matrix}'
            )
            if hasattr(self, '_calibration_timeout'):
                self._calibration_timeout.cancel()

    def _calibration_fallback(self):
        if self.camera_matrix is not None:
            return
        self._calibration_timeout.cancel()
        fx = 381.0
        fy = 381.0
        cx = 320.0
        cy = 240.0
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.get_logger().warn(
            f'Camera calibration not received in 3s, using fallback:\n'
            f'{self.camera_matrix}\n'
            f'Set camera_matrix_use_override:=true to use custom values'
        )

    def control_callback(self):
        if self.state == 'SEARCHING':
            twist = Twist()
            twist.angular.z = self.spin_speed
            self.cmd_vel_pub.publish(twist)

        elif self.state == 'NAVIGATING' and not self._nav_goal_sent:
            elapsed = (self.get_clock().now() - self._nav_start_time).nanoseconds / 1e9
            if elapsed > 5.0:
                self.get_logger().warn('Timed out sending Nav2 goal, returning to search')
                self._target_marker_id = None
                self._nav_goal_sent = False
                self.state = 'SEARCHING'
                return
            self._try_send_nav_goal()

    def image_callback(self, msg):
        if self.state not in ('SEARCHING', 'NAVIGATING'):
            return
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        self._current_image_frame = msg.header.frame_id

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            frame_id = self._current_image_frame
            for i, marker_id in enumerate(ids.flatten()):
                self._publish_marker_tf(rvecs[i][0], tvecs[i][0], int(marker_id), frame_id)

            if self.state == 'SEARCHING':
                for i, marker_id in enumerate(ids.flatten()):
                    int_id = int(marker_id)
                    if int_id in self.visited_markers:
                        continue
                    if int_id == self._target_marker_id:
                        continue
                    if tvecs[i][0][2] <= 0:
                        continue
                    self._target_marker_id = int_id
                    self.visited_markers.add(int_id)
                    self._nav_start_time = self.get_clock().now()
                    self.state = 'NAVIGATING'
                    self._try_send_nav_goal()
                    return

    def _try_send_nav_goal(self):
        mid = self._target_marker_id
        if mid is None:
            self.get_logger().warn('_try_send_nav_goal: _target_marker_id is None, skipping')
            self.state = 'SEARCHING'
            return

        marker_frame = f'aruco_marker_{int(mid)}'

        # 1) Try live TF lookup (marker currently visible)
        t = None
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal_frame, marker_frame,
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=0.3)
            )
        except Exception:
            pass

        # 2) Fall back to saved map-frame position
        if t is None and int(mid) in self.marker_positions:
            pos = self.marker_positions[int(mid)]
            self.get_logger().info(
                f'Using saved position for marker {int(mid)}: ({pos["x"]:.2f}, {pos["y"]:.2f})'
            )
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = self.goal_frame
            goal_msg.pose.pose.position.x = pos['x']
            goal_msg.pose.pose.position.y = pos['y']
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.w = 1.0

            self.get_logger().info(
                f'Sending Nav2 goal to marker {int(mid)} '
                f'({pos["x"]:.2f}, {pos["y"]:.2f}) [saved]'
            )

            if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().warn('Nav2 action server not ready')
                return

            send_future = self.nav_action_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self._goal_response_callback)
            self._nav_goal_sent = True
            self.state = 'NAVIGATING'
            return

        # 3) If no saved position either, give up
        if t is None:
            self.get_logger().warn(
                f'No position known for marker {int(mid)} (not visible, no saved pose)'
            )
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not ready')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.pose.position.x = t.transform.translation.x
        goal_msg.pose.pose.position.y = t.transform.translation.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal to marker {int(mid)} '
            f'({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f})'
        )

        send_future = self.nav_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)
        self._nav_goal_sent = True
        self.state = 'NAVIGATING'

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 goal for marker {self._target_marker_id} was rejected')
            self._target_marker_id = None
            self._nav_goal_sent = False
            self.state = 'SEARCHING'
            return

        self._goal_handle = goal_handle
        self.get_logger().info(f'Nav2 goal accepted, navigating to marker {self._target_marker_id}')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        result = future.result()
        self._goal_handle = None
        self._nav_goal_sent = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Reached marker {self._target_marker_id}! Waiting 2s...'
            )
        else:
            self.get_logger().warn(
                f'Navigation to marker {self._target_marker_id} '
                f'failed with status {result.status}'
            )

        self.state = 'WAITING'
        self.wait_timer = self.create_timer(2.0, self._wait_done_callback)

    def _wait_done_callback(self):
        self.wait_timer.cancel()

        if len(self.visited_markers) >= self.total_markers:
            self.state = 'COMPLETE'
            self.get_logger().info(
                f'All {self.total_markers} markers visited! '
                f'Visited: {sorted(self.visited_markers)}'
            )
            return

        self.state = 'SEARCHING'
        self._target_marker_id = None
        self.get_logger().info(
            f'Resuming search. Visited {len(self.visited_markers)}/{self.total_markers}: '
            f'{sorted(self.visited_markers)}'
        )

    def _publish_marker_tf(self, rvec, tvec, marker_id, frame_id=None):
        R_cv, _ = cv2.Rodrigues(rvec)
        R_ros = self.R_cv_to_ros @ R_cv
        t_ros = self.R_cv_to_ros @ tvec

        if not hasattr(self, '_last_debug_time'):
            self._last_debug_time = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._last_debug_time).nanoseconds / 1e9
        if elapsed > 3.0:
            dist = np.linalg.norm(t_ros)
            self.get_logger().info(
                f'Marker {marker_id}: distance={dist:.3f}m '
                f't_ros=({t_ros[0]:.3f}, {t_ros[1]:.3f}, {t_ros[2]:.3f}) '
                f'frame={frame_id or self.parent_frame}'
            )
            self._last_debug_time = self.get_clock().now()

        trace = R_ros[0, 0] + R_ros[1, 1] + R_ros[2, 2]
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            qx = (R_ros[2, 1] - R_ros[1, 2]) / s
            qy = (R_ros[0, 2] - R_ros[2, 0]) / s
            qz = (R_ros[1, 0] - R_ros[0, 1]) / s
            qw = s / 4.0
        elif R_ros[0, 0] > R_ros[1, 1] and R_ros[0, 0] > R_ros[2, 2]:
            s = math.sqrt(1.0 + R_ros[0, 0] - R_ros[1, 1] - R_ros[2, 2]) * 2.0
            qx = s / 4.0
            qy = (R_ros[0, 1] + R_ros[1, 0]) / s
            qz = (R_ros[0, 2] + R_ros[2, 0]) / s
            qw = (R_ros[2, 1] - R_ros[1, 2]) / s
        elif R_ros[1, 1] > R_ros[2, 2]:
            s = math.sqrt(1.0 + R_ros[1, 1] - R_ros[0, 0] - R_ros[2, 2]) * 2.0
            qx = (R_ros[0, 1] + R_ros[1, 0]) / s
            qy = s / 4.0
            qz = (R_ros[1, 2] + R_ros[2, 1]) / s
            qw = (R_ros[0, 2] - R_ros[2, 0]) / s
        else:
            s = math.sqrt(1.0 + R_ros[2, 2] - R_ros[0, 0] - R_ros[1, 1]) * 2.0
            qx = (R_ros[0, 2] + R_ros[2, 0]) / s
            qy = (R_ros[1, 2] + R_ros[2, 1]) / s
            qz = s / 4.0
            qw = (R_ros[1, 0] - R_ros[0, 1]) / s

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id if frame_id else self.parent_frame
        t.child_frame_id = f'aruco_marker_{marker_id}'
        t.transform.translation.x = float(t_ros[0])
        t.transform.translation.y = float(t_ros[1])
        t.transform.translation.z = float(t_ros[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)

        marker_frame = f'aruco_marker_{marker_id}'
        try:
            map_tf = self.tf_buffer.lookup_transform(
                self.goal_frame, marker_frame,
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=0.2)
            )
            self.marker_positions[marker_id] = {
                'x': map_tf.transform.translation.x,
                'y': map_tf.transform.translation.y,
            }
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
