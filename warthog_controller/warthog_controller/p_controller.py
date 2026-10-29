import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # Required to enable buffer.transform() for PoseStamped


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q) -> float:
    """Convert a quaternion into Euler yaw angle in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class Controller(Node):
    """Odometry-based proportional (P) controller for mobile robot navigation."""

    def __init__(self):
        super().__init__('p_controller_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare parameters
        self.declare_parameter('cmd_vel_topic', 'warthog/cmd_vel_step')
        self.declare_parameter('odom_topic', '/warthog/fast_lio/odometry/imu')
        self.declare_parameter('goal_topic', '/warthog/gecko_inspection/view_plan')
        self.declare_parameter('new_pos_topic', 'warthog/new_position')
        self.declare_parameter('reached_topic', 'warthog/reached')
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('dist_tolerance', 0.2)
        self.declare_parameter('standoff_distance', 1.40)
        self.declare_parameter('yaw_tolerance', 0.1)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('base_frame', 'warthog/base_link')

        # Retrieve parameter values
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.new_pos_topic = self.get_parameter('new_pos_topic').value
        reached_topic = self.get_parameter('reached_topic').value
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.dist_tolerance = float(self.get_parameter('dist_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        control_rate = float(self.get_parameter('control_rate').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.standoff_distance = float(self.get_parameter('standoff_distance').value)

        # Publishers
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.reached_publisher = self.create_publisher(Bool, reached_topic, 10)

        # Subscribers: primary odometry and fallback odometry
        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        if self.odom_topic != '/warthog/fast_lio/odometry/imu':
            self.fallback_odom_sub = self.create_subscription(
                Odometry, '/warthog/fast_lio/odometry/imu', self.fallback_odom_callback, 10
            )

        # Goal subscriptions: supports both mission_control goal_topic and legacy new_pos_topic
        self.goal_subscriber = self.create_subscription(
            PoseArray, self.goal_topic, self.goal_callback, 10
        )
        if self.new_pos_topic != self.goal_topic:
            self.new_pos_subscriber = self.create_subscription(
                PoseStamped, self.new_pos_topic, self.goal_callback, 10
            )

        timer_period = 1.0 / control_rate if control_rate > 0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_frame = 'odom'
        self.odom_received = False
        self.primary_odom_received = False

        # Goal state in odometry coordinate frame
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None
        self.goal_reached = False
        self.goal_active = False
        self.stop_pub_count = 0
        self.pending_goal: Optional[PoseStamped] = None

        self.get_logger().info(
            f'P-Controller active. cmd_vel: {cmd_vel_topic}, odom: {self.odom_topic}, '
            f'goals: {self.goal_topic}'
        )

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates from primary topic."""
        self.primary_odom_received = True
        self.update_odometry(msg)

    def fallback_odom_callback(self, msg: Odometry):
        """Handle odometry updates from fallback topic if primary has no data."""
        if not self.primary_odom_received:
            self.update_odometry(msg)

    def update_odometry(self, msg: Odometry):
        """Update robot pose state from odometry message."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'Odometry connected ({self.odom_frame}). '
                f'Initial pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}'
            )
            if self.pending_goal is not None:
                goal = self.pending_goal
                self.pending_goal = None
                self.process_goal(goal)

    def goal_callback(self, msg: PoseArray):
        """Receive goal pose message."""
        # 1. Safety check: make sure the array isn't empty to prevent IndexError
        if not msg.poses:
            self.get_logger().warn('Received an empty PoseArray, ignoring.')
            return

        # 2. Package the array's header and the final Pose into a PoseStamped object
        target_pose = PoseStamped()
        target_pose.header = msg.header
        target_pose.pose = msg.poses[-1]

        # 3. Cache or process as usual
        if not self.odom_received:
            self.get_logger().warn(
                'Goal received before odometry; caching until odometry is available.'
            )
            self.pending_goal = target_pose
            return

        self.process_goal(target_pose)

    def process_goal(self, msg: PoseStamped):
        """Transform and store goal pose in odometry frame using TF2."""
        if not msg.header.frame_id:
            self.get_logger().warn('Goal received with empty frame_id! Assuming odometry frame.')
            msg.header.frame_id = self.odom_frame

        try:
            # Let tf2 handle the math, no matter what frame the goal is in (map, base_link, camera, etc.)
            # The timeout ensures it waits briefly if the tf tree is slightly lagging
            transformed_msg = self.tf_buffer.transform(
                msg,
                self.odom_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract the fully transformed global coordinates
            self.goal_x = transformed_msg.pose.position.x
            self.goal_y = transformed_msg.pose.position.y
            self.goal_yaw = get_yaw_from_quaternion(transformed_msg.pose.orientation)

            self.get_logger().info(
                f"Transformed goal from '{msg.header.frame_id}' to '{self.odom_frame}': "
                f"x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f}"
            )

            self.goal_reached = False
            self.goal_active = True
            self.stop_pub_count = 0

        except TransformException as ex:
            self.get_logger().error(f'Failed to transform goal from {msg.header.frame_id} to {self.odom_frame}: {ex}')
            # The goal is dropped, but the node survives and waits for a valid goal
            return

    def timer_callback(self):
        """Execute control loop iteration."""
        if not self.goal_active or self.goal_x is None or not self.odom_received:
            return

        # Calculate Cartesian errors in odometry frame
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_error = math.hypot(dx, dy)

        effective_dist = dist_error - self.standoff_distance

        # PHASE 1: Position control (drive towards target coordinate)
        # Use effective_dist instead of dist_error
        if effective_dist > self.dist_tolerance:
            target_angle = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_angle - self.yaw)

            # Angular speed
            z_speed = self.kp_angular * yaw_error
            z_speed = max(min(z_speed, self.max_angular_speed), -self.max_angular_speed)

            # Pivot in place first if facing more than 45 degrees away
            if abs(yaw_error) > math.pi / 4:
                x_speed = 0.0
            else:
                # Use effective_dist for P-control to decelerate smoothly as it reaches the standoff radius
                x_speed = self.kp_linear * effective_dist * max(0.0, math.cos(yaw_error))
                x_speed = max(min(x_speed, self.max_linear_speed), -self.max_linear_speed)

            self.publish_cmd(x_speed, z_speed)

        # PHASE 2: Position reached, align to target yaw
        else:
            yaw_error = normalize_angle(self.goal_yaw - self.yaw)

            if abs(yaw_error) > self.yaw_tolerance:
                x_speed = 0.0
                z_speed = self.kp_angular * yaw_error
                z_speed = max(min(z_speed, self.max_angular_speed), -self.max_angular_speed)
                self.publish_cmd(x_speed, z_speed)
            else:
                # Goal fully reached
                self.publish_cmd(0.0, 0.0)

                if not self.goal_reached:
                    self.goal_reached = True
                    self.get_logger().info(
                        f'Target reached! Pose: ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})'
                    )
                    reached_msg = Bool()
                    reached_msg.data = True
                    self.reached_publisher.publish(reached_msg)

                self.stop_pub_count += 1
                if self.stop_pub_count >= 5:
                    self.goal_active = False
                    self.get_logger().info('P-Controller idling until next goal.')

    def publish_cmd(self, x: float, z: float):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(z)
        self.publisher.publish(twist)

    def destroy_node(self):
        """Stop robot on node shutdown."""
        try:
            self.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Run p_controller node."""
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down p_controller.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
