import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

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
        super().__init__('controller_node')

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        # Increased timer rate to 10Hz (0.1s) for smoother P-control
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Proportional (P) Controller Gains
        self.kp_linear = 0.005   # Gain for Cartesian/Distance control
        self.kp_angular = 0.01  # Gain for Yaw/Angular control

        # Tolerances for when to stop
        self.dist_tolerance = 0.05
        self.yaw_tolerance = 0.05 # roughly 3 degrees

        # Current State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goal State
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.msg_callback, 10)
        self.new_pos_subscriber = self.create_subscription(PoseStamped, '/new_position', self.new_pos_callback, 1)

    def msg_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)

    def new_pos_callback(self, msg):
        # Extract both position and orientation from the target
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = get_yaw_from_quaternion(msg.pose.orientation)
        self.get_logger().info(f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f}")

    def timer_callback(self):
        new_msg = TwistStamped()

        if self.goal_x is None:
            return

        # Calculate Cartesian errors
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_error = math.hypot(dx, dy)

        x_speed = 0.0
        z_speed = 0.0

        # PHASE 1: Go to the (x, y) target coordinate
        if dist_error > self.dist_tolerance:
            # Angle to the target coordinate
            target_angle = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_angle - self.yaw)

            # Angular P-Controller
            z_speed = self.kp_angular * yaw_error

            # Linear P-Controller
            x_speed = self.kp_linear * dist_error

            # Cap maximum speeds for safety
            x_speed = min(x_speed, 0.5)
            z_speed = max(min(z_speed, 1.0), -1.0)

            # Fix: If the robot is facing the wrong way, pivot first before driving forward
            if abs(yaw_error) > math.pi / 4: # 45 degrees
                x_speed = 0.0
        if dist_error > self.dist_tolerance:
            # Angle to the target coordinate
            target_angle = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_angle - self.yaw)

            # Angular P-Controller
            z_speed = self.kp_angular * yaw_error

            # Linear P-Controller
            x_speed = self.kp_linear * dist_error

            # Cap maximum speeds for safety
            x_speed = min(x_speed, 0.5)
            z_speed = max(min(z_speed, 1.0), -1.0)

            # Fix: If the robot is facing the wrong way, pivot first before driving forward
            if abs(yaw_error) > math.pi / 4: # 45 degrees
                x_speed = 0.0

        # PHASE 2: Reached the (x, y) target, now align to the target Yaw
        else:
            yaw_error = normalize_angle(self.goal_yaw - self.yaw)

            if abs(yaw_error) > self.yaw_tolerance:
                # Angular P-Controller based on final target yaw
                x_speed = 0.0
                z_speed = self.kp_angular * yaw_error
                z_speed = max(min(z_speed, 1.0), -1.0) # Cap angular speed
            else:
                # Goal fully reached
                x_speed = 0.0
                z_speed = 0.0

        new_msg.twist.linear.x = x_speed
        new_msg.twist.angular.z = z_speed

        self.publisher.publish(new_msg)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
