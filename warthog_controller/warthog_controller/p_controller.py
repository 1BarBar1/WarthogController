import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    """Keeps an angle mapped between -pi and pi."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def get_yaw_from_quaternion(q):
    """Converts a quaternion into Euler yaw."""
    return np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class Controller(Node):
    def __init__(self):
        super().__init__('p_controller_node')

        # Declare parameters
        self.declare_parameter('cmd_vel_topic', 'warthog/cmd_vel_step')
        self.declare_parameter('odom_topic', 'warthog/odom')
        self.declare_parameter('new_pos_topic', 'warthog/new_position')
        self.declare_parameter('reached_topic', 'warthog/reached')
        self.declare_parameter('kp_linear', 0.2)
        self.declare_parameter('kp_angular', 0.5)
        self.declare_parameter('dist_tolerance', 0.2)
        self.declare_parameter('yaw_tolerance', 0.2)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('base_frame', 'warthog/base_link')

        # Retrieve parameters
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        new_pos_topic = self.get_parameter('new_pos_topic').value
        reached_topic = self.get_parameter('reached_topic').value
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.dist_tolerance = float(self.get_parameter('dist_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        control_rate = float(self.get_parameter('control_rate').value)
        self.base_frame = self.get_parameter('base_frame').value

        # Publishers and Subscribers
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.reached_publisher = self.create_publisher(Bool, reached_topic, 10)
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.msg_callback, 10)
        self.new_pos_subscriber = self.create_subscription(PoseStamped, new_pos_topic, self.new_pos_callback, 1)

        timer_period = 1.0 / control_rate if control_rate > 0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Current Robot State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goal State
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.goal_reached = False 
        self.sature = 0.2
        self.sature_ang = 0.2
        self.dist = 0.90

        self.get_logger().info(f"P-Controller initialized. Controlling on {cmd_vel_topic}, listening to {new_pos_topic}")

    def msg_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)

    def new_pos_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = get_yaw_from_quaternion(msg.pose.orientation)
        self.goal_reached = False
        # self.get_logger().info(
        #     f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f}"
        # )

    def timer_callback(self):
        if self.goal_x is None:
            return

        # Calculate Cartesian distance errors
        dx = self.goal_x
        dy = self.goal_y 
        print(f'goal_x {dx}')
        dx = dx - self.dist
        print(f'goal after {dx}')
        print(f'dy {dy}')
        
        dist_error = math.hypot(dx, dy)
        print(f'dist error {dist_error}')
        x_speed = 0.0
        z_speed = 0.0

        # PHASE 1: Drive towards the (x, y) target coordinate
        if dist_error > self.dist_tolerance:
            target_angle = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_angle - self.yaw)

            # Angular P-Controller
            z_speed = self.kp_angular * yaw_error

            # Linear P-Controller
            x_speed = self.kp_linear * dist_error

            # Cap maximum speeds for safety
            x_speed = min(x_speed, self.max_linear_speed)
            z_speed = max(min(z_speed, self.max_angular_speed), -self.max_angular_speed)

            # Pivot in place first if facing more than 45 degrees away from target
            if abs(yaw_error) > math.pi / 4:
                x_speed = 0.0

        # PHASE 2: (x, y) target reached, now align to target yaw
        else:
            yaw_error = normalize_angle(self.goal_yaw - self.yaw)

            if abs(yaw_error) > self.yaw_tolerance:
                x_speed = 0.0
                z_speed = self.kp_angular * yaw_error
                z_speed = max(min(z_speed, self.max_angular_speed), -self.max_angular_speed)
            else:
                # Goal fully reached
                x_speed = 0.0
                z_speed = 0.0
                if not self.goal_reached:
                    self.goal_reached = True
                    #self.get_logger().info("Target position and orientation reached!")
                    reached_msg = Bool()
                    reached_msg.data = True
                    self.reached_publisher.publish(reached_msg)

        # if x_speed > self.sature:
        #     x_speed = self.sature
        # elif x_speed < -self.sature:
        #     x_speed = -self.sature

        # if z_speed > self.sature_ang or z_speed < -self.sature_ang:
        #     z_speed = self.sature_ang

        
        x_speed = np.clip(x_speed, -self.sature, self.sature)
        z_speed = np.clip(z_speed, -self.sature_ang, self.sature_ang)
        # Publish TwistStamped
        new_msg = Twist()
        # new_msg.header.stamp = self.get_clock().now().to_msg()
        # new_msg.header.frame_id = self.base_frame
        new_msg.linear.x = x_speed
        new_msg.angular.z = z_speed

        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down p_controller.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
