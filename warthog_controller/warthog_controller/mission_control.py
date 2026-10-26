import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class MissionLog:
    def __init__(self, mission_states: dict):
        self.mission_states = mission_states

    def current_mission(self) -> Optional[str]:
        for state in self.mission_states:
            if self.mission_states[state] != "done":
                return state
        return None

    def check_done_mission(self):
        for state in self.mission_states:
            if self.mission_states[state] != "done":
                self.mission_states[state] = "done"
                break

    def next_mission(self) -> Optional[str]:
        for state in self.mission_states:
            if self.mission_states[state] != "done":
                return self.mission_states[state]
        return None


class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control")

        # Declare parameters
        self.declare_parameter("exploration_trigger", "warthog/exploration_trigger")
        self.declare_parameter("reached_topic", "warthog/reached")
        self.declare_parameter("goal_topic", "warthog/goal_pose")
        self.declare_parameter("camera_feedback", "warthog/poses")
        self.declare_parameter("goal_ralative_to_camera", "warthog/new_position")
        self.declare_parameter("odom_topic", "warthog/odom")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("continuous_visual_tracking", False)
        self.declare_parameter("standoff_distance", 0.5)  # Distance to maintain from detected object (m)

        missions = {"visual_tune": ""}
        self.missionLog = MissionLog(missions)

        # Get parameter values
        explore_topic = self.get_parameter("exploration_trigger").value
        reached_topic = self.get_parameter("reached_topic").value
        self.auto_start = self.get_parameter("auto_start").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.camera_topic = self.get_parameter("camera_feedback").value
        self.goal_relative_to_camera_topic = self.get_parameter("goal_ralative_to_camera").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.continuous_visual_tracking = self.get_parameter("continuous_visual_tracking").value
        self.standoff_distance = float(self.get_parameter("standoff_distance").value)

        # TF Buffer and Listener for accurate coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and Subscribers
        self.exploration_publisher = self.create_publisher(
            Bool, explore_topic, 10
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, self.goal_topic, 10
        )
        self.goal_relative_to_camera_publisher = self.create_publisher(
            PoseStamped, self.goal_relative_to_camera_topic, 10
        )

        self.finished_subscription = self.create_subscription(
            Bool, reached_topic, self.finished_callback, 10
        )
        self.camera_subscription = self.create_subscription(
            PoseArray, self.camera_topic, self.camera_feedback_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # Track robot state
        self.is_exploring = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_frame = "warthog/odom"

        self.get_logger().info("Mission Control initialized.")

        # Auto-start after a 1-second delay
        if self.auto_start:
            self.start_timer = self.create_timer(1.0, self.start_robot)

    def get_yaw_from_quaternion(self, q):
        """Converts a quaternion into Euler yaw."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def start_robot(self):
        self.start_timer.cancel()
        current = self.missionLog.current_mission()
        if current == "explore":
            self.trigger_exploration(enable=True)
        elif current == "visual_tune":
            self.get_logger().info("Mission Control started in visual_tune mode, waiting for camera feedback...")
        elif current is not None:
            val = self.missionLog.next_mission()
            if val and val != "":
                x, y, yaw = map(float, val.split(','))
                self.publish_goal(x, y, yaw)

    def trigger_exploration(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.exploration_publisher.publish(msg)

        self.is_exploring = enable
        state_str = "Started" if enable else "Stopped"
        self.get_logger().info(f"{state_str} exploration.")

    def finished_callback(self, msg: Bool):
        if not msg.data:
            return

        current = self.missionLog.current_mission()
        if self.is_exploring:
            self.get_logger().info(">>> The robot has finished exploring! <<<")
            self.trigger_exploration(enable=False)
            self.missionLog.check_done_mission()
        elif current is not None:
            self.get_logger().info(f">>> The robot finished mission stage '{current}'! <<<")
            self.missionLog.check_done_mission()

        next_stage = self.missionLog.current_mission()
        self.get_logger().info(f"Next mission stage: {next_stage}")
        if next_stage is None:
            self.get_logger().info("All missions complete.")
            return

        next_goal = self.missionLog.next_mission()
        if next_goal and next_goal != "":
            x, y, yaw = map(float, next_goal.split(','))
            self.publish_goal(x, y, yaw)
        elif next_stage == "visual_tune":
            self.get_logger().info("Switched to visual_tune mode, waiting for camera feedback...")

    def publish_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "warthog/base_link"

        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal at X: {x}, Y: {y}")

    def camera_feedback_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            return

        current = self.missionLog.current_mission()
        # Only react when in visual_tune stage (or when no conflicting mission)
        if current is not None and current != "visual_tune":
            return

        detected_pose: Pose = msg.poses[0]
        self.get_logger().info(
            f"Camera detected target at X: {detected_pose.position.x:.2f}, "
            f"Y: {detected_pose.position.y:.2f}, Z: {detected_pose.position.z:.2f}. "
            "Stopping current controller and starting p_controller!"
        )

        # 1. Stop current controller / exploration
        if self.is_exploring:
            self.trigger_exploration(enable=False)

        # 2. Advance mission state if not continuous tracking
        if not self.continuous_visual_tracking and current == "visual_tune":
            self.missionLog.check_done_mission()

        # 3. Compute target coordinates with standoff distance and dispatch to p_controller
        self.goal_camera(detected_pose, msg.header)

    def goal_camera(self, detected_pose: Pose, header):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        target_frame = self.odom_frame if self.odom_frame else "warthog/odom"
        goal_msg.header.frame_id = target_frame

        transformed = False
        frame_id = header.frame_id if header.frame_id else ""
        if frame_id:
            try:
                pose_cam = PoseStamped()
                pose_cam.header = header
                pose_cam.pose = detected_pose
                if self.tf_buffer.can_transform(target_frame, frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.1)):
                    pose_out = self.tf_buffer.transform(pose_cam, target_frame)
                    obj_x = pose_out.pose.position.x
                    obj_y = pose_out.pose.position.y
                    transformed = True
            except Exception as e:
                self.get_logger().warn(
                    f"TF transform from '{frame_id}' to '{target_frame}' failed: {e}. Falling back to planar projection."
                )

        if not transformed:
            # Fallback planar projection:
            # Optical frame: X is right, Y is down, Z is forward depth
            if "optical" in frame_id.lower():
                rel_forward = detected_pose.position.z
                rel_lateral = -detected_pose.position.x
            else:
                rel_forward = detected_pose.position.x
                rel_lateral = detected_pose.position.y

            obj_x = self.current_x + (rel_forward * math.cos(self.current_yaw) - rel_lateral * math.sin(self.current_yaw))
            obj_y = self.current_y + (rel_forward * math.sin(self.current_yaw) + rel_lateral * math.cos(self.current_yaw))

        # Vector from current robot position to the detected object
        dx = obj_x - self.current_x
        dy = obj_y - self.current_y
        dist_to_obj = math.hypot(dx, dy)

        # Apply standoff distance so the robot stops in front of the object
        target_yaw = math.atan2(dy, dx)
        if dist_to_obj > self.standoff_distance and dist_to_obj > 0.001:
            ratio = (dist_to_obj - self.standoff_distance) / dist_to_obj
            goal_x = self.current_x + dx * ratio
            goal_y = self.current_y + dy * ratio
        else:
            # Robot is already at or within standoff distance
            goal_x = self.current_x
            goal_y = self.current_y

        goal_msg.pose.position.x = float(goal_x)
        goal_msg.pose.position.y = float(goal_y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.goal_relative_to_camera_publisher.publish(goal_msg)
        self.get_logger().info(
            f"Object at X: {obj_x:.2f}, Y: {obj_y:.2f}. "
            f"Published standoff goal ({target_frame}) -> X: {goal_x:.2f}, Y: {goal_y:.2f}, Yaw: {target_yaw:.2f}"
        )

    def odom_callback(self, msg: Odometry):
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down mission control.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
