
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
import math

# 1. Import PoseStamped
from geometry_msgs.msg import PoseStamped

class MissionLog():
    def __init__(self,mission_states:dict):
        self.mission_states = mission_states

    def check_done_mission(self):
        for state in self.mission_states:
            if self.mission_states[state] != "done":
                self.mission_states[state] = "done"
                break
    def next_mission(self):
        for state in self.mission_states:
            if self.mission_states[state] != "done":
                return self.mission_states[state]
        return None

class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control")

        # Declare parameters
        self.declare_parameter("exploration_trigger", "/exploration_trigger")
        self.declare_parameter("reached_topic", "/reached")
        self.declare_parameter("goal_topic", "/goal_pose") 
        self.declare_parameter("camera_feedback","/poses")
        self.declare_parameter("goal_ralative_to_camera", "/new_position")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("auto_start", True)

        missions = {"explore":"", "go_home":"0.0,0.0,0.0", "go_to_mission":"4.0,0.5,0.0","visual_tune":""}
        self.missionLog = MissionLog(missions)

        # Get parameter values
        explore_topic = self.get_parameter("exploration_trigger").value
        reached_topic = self.get_parameter("reached_topic").value
        self.auto_start = self.get_parameter("auto_start").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.camera_topic = self.get_parameter("camera_feedback").value
        self.goal_relative_to_camera_topic = self.get_parameter("goal_ralative_to_camera").value
        self.odom_topic = self.get_parameter("odom_topic").value

        # Create the publishers and subscribers
        self.exploration_publisher = self.create_publisher(
            Bool, explore_topic, 10
        )

        # 2. Change Pose to PoseStamped
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

        # Track the status of the robot
        self.is_exploring = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 1.0

        self.get_logger().info("Mission Control initialized for single robot.")

        # Auto-start exploration after a 1-second delay
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
        self.trigger_exploration(enable=True)
        self.missionLog.check_done_mission()

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

        if self.is_exploring:
            self.get_logger().info(">>> The robot has finished exploring! <<<")
            self.trigger_exploration(enable=False)
        else:
            self.get_logger().info(">>> The robot reached its mission waypoint! <<<")

        next_mission = self.missionLog.next_mission()
        self.get_logger().info(f"next goal: {next_mission}")
        if next_mission is None:
            self.get_logger().info("All missions complete.")
            return

        if next_mission != "":
            x, y, yaw = map(float, next_mission.split(','))
            self.publish_goal(x, y, yaw)
            self.missionLog.check_done_mission()

            # Example of triggering a goal once exploration finishes:



    # 3. Implement the publish_goal method
    def publish_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = PoseStamped()

        # Set the header
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # Standard global frame for Nav2

        # Set position
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0

        # Convert yaw (2D rotation) to a quaternion for Nav2
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

        # Publish
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published Nav2 goal at X: {x}, Y: {y}")

    def camera_feedback_callback(self, msg: PoseArray):
        # Ignore empty PoseArrays when nothing is detected
        if len(msg.poses) == 0:
            return

        # Optional: only react when we have reached the "visual_tune" stage
        if self.missionLog.next_mission() != "":
            return

        detected_pose: Pose = msg.poses[0]
        self.get_logger().info(
            f"Camera detected target at X: {detected_pose.position.x:.2f}, "
            f"Y: {detected_pose.position.y:.2f}. Stopping path planner!"
        )

        # Send False to stop PathPlannerNode and clear the active path
        self.trigger_exploration(enable=False)
        self.missionLog.check_done_mission()

        goal_x = msg.poses[0].position.x
        goal_y = msg.poses[0].position.y
        goal_yaw = self.get_yaw_from_quaternion(msg.poses[0].orientation)
        self.goal_camera(goal_x, goal_y, goal_yaw)


    def goal_camera(self, camera_x: float, camera_y: float, camera_yaw: float):
        # Create a new PoseStamped message for the goal relative to the camera
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # Assuming the odometry is in the "map" frame

        # 1. Apply 2D Rotation Matrix to translate camera frame to global frame
        global_x = self.current_x + (camera_x * math.cos(self.current_yaw) - camera_y * math.sin(self.current_yaw))
        global_y = self.current_y + (camera_x * math.sin(self.current_yaw) + camera_y * math.cos(self.current_yaw))

        # 2. Add the relative yaw to the robot's current global yaw
        global_yaw = self.current_yaw + camera_yaw

        # Set the absolute position
        goal_msg.pose.position.x = global_x
        goal_msg.pose.position.y = global_y
        goal_msg.pose.position.z = 0.0

        # Set the absolute orientation
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(global_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(global_yaw / 2.0)

        # Publish the new global goal for the controller
        self.goal_relative_to_camera_publisher.publish(goal_msg)

        self.get_logger().info(f"Published target to controller -> X:{global_x:.2f}, Y:{global_y:.2f}, Yaw:{global_yaw:.2f}")

    def odom_callback(self, msg: Odometry):
        # Optional: only react when we have reached the "visual_tune" stage
        if self.missionLog.next_mission() != "":
            return

        # Extract the robot's current position and orientation from the Odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = math.atan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                   msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
            1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 + msg.pose.pose.orientation.z ** 2)
        )



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
