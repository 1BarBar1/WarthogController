#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from typing import Optional
import math
import subprocess

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

        # Parameters
        self.declare_parameter("exploration_trigger", "warthog/exploration_trigger")
        self.declare_parameter("visual_servo_trigger", "warthog/visual_servo_trigger")
        self.declare_parameter("reached_topic", "warthog/reached")
        self.declare_parameter("goal_topic", "warthog/goal_pose")
        self.declare_parameter("camera_feedback", "warthog/poses")
        self.declare_parameter("path_feedback", "warthog/plan")
        self.declare_parameter("auto_start", True)
        # "wating":"","Pathing":""
        # Change this line in __init__:
        missions = {"visual_tune": "", "hello_moveit": ""}
        self.missionLog = MissionLog(missions)

        explore_topic = self.get_parameter("exploration_trigger").value
        vs_trigger_topic = self.get_parameter("visual_servo_trigger").value
        reached_topic = self.get_parameter("reached_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.camera_topic = self.get_parameter("camera_feedback").value
        self.auto_start = self.get_parameter("auto_start").value
        self.path_topic = self.get_parameter("path_feedback").value

        # --- Mission Publishers & Subscribers ---
        self.exploration_publisher = self.create_publisher(Bool, explore_topic, 10)
        self.vs_trigger_publisher = self.create_publisher(Bool, vs_trigger_topic, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.finished_subscription = self.create_subscription(Bool, reached_topic, self.finished_callback, 10)
        self.camera_subscription = self.create_subscription(PoseArray, self.camera_topic, self.camera_feedback_callback, 10)
        self.path_subscription = self.create_subscription(Path, self.path_topic, self.path_feedback_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'warthog/odom', self.odom_callback, 10)

        self.nav_cancel_client = self.create_client(CancelGoal, 'warthog/navigate_to_pose/_action/cancel_goal')

        self.current_x = None
        self.current_y = None
        self.distance_tolerance = 0.5 # Meters tolerance

        self.is_exploring = False
        self.is_visual_servoing = False
        self.path_enable = False

        # --- Trigger Manager Additions ---
        self.start_trigger_sub = self.create_subscription(Bool, "warthog/start_trigger", self.trigger_callback, 10)
        self._arm_r_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "warthog/ur5e_manipulator_r_controller/follow_joint_trajectory",
        )
        self.process_oden = None
        self.process_moveit = None
        self.is_processing = False

        self.get_logger().info("============================================================")
        self.get_logger().info("Mission Control Node Active!")
        self.get_logger().info("Send 'ros2 topic pub --once warthog/start_trigger std_msgs/msg/Bool \"{data: true}\"' to trigger arm.")
        self.get_logger().info("============================================================")

        if self.auto_start:
            self.start_timer = self.create_timer(1.0, self.start_robot)

    # ---------------------------------------------------------
    # TRIGGER MANAGER LOGIC
    # ---------------------------------------------------------
    def trigger_callback(self, msg: Bool):
        if msg.data and not self.is_processing:
            self.is_processing = True
            self.get_logger().info("TRIGGER RECEIVED! Step 1: Moving Arm R to gripping_pose...")
            self.send_arm_r_home_goal()
        elif self.is_processing:
            self.get_logger().warn("Pipeline is already executing. Trigger ignored.")

    def send_arm_r_home_goal(self):


        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            "arm_r_shoulder_pan_joint",
            "arm_r_shoulder_lift_joint",
            "arm_r_elbow_joint",
            "arm_r_wrist_1_joint",
            "arm_r_wrist_2_joint",
            "arm_r_wrist_3_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [-1.5359, -1.885, 2.1293, -4.8171, -0.2793, 1.4835]
        point.time_from_start = Duration(sec=5, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending trajectory goal for Arm R gripping_pose...")
        send_goal_future = self._arm_r_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm R gripping_pose goal rejected by controller!")
            self.is_processing = False
            return

        self.get_logger().info("Arm R goal accepted. Moving Arm R to gripping_pose...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Arm R has reached gripping_pose successfully!")
        self.get_logger().info("Step 2: Launching 'oden publisher' and 'hello_moveit pick_place_ur5e'...")
        self.launch_scripts()

    def launch_scripts(self):

        moveit_workspace_setup = "/home/warthog/Public/harry_summerjob/robobuilder_native/Robobuilder_moveit2_perception/install/setup.bash"

        # Chain the source command and the ros2 launch command together
        cmd_moveit = f"source {moveit_workspace_setup} && ros2 launch hello_moveit pick_place_ur5e.launch.py"

        # Note: We use shell=True and specify bash so the 'source' command works properly
        self.process_moveit = subprocess.Popen(cmd_moveit, shell=True, executable='/bin/bash')
        self.get_logger().info(f"Started hello_moveit (PID {self.process_moveit.pid}) from external workspace")

        # Finalize the mission log
        self.missionLog.check_done_mission()
        self.get_logger().info("All missions complete!")

    # ---------------------------------------------------------
    # MISSION CONTROL LOGIC
    # ---------------------------------------------------------
    def start_robot(self):
        self.start_timer.cancel()
        current = self.missionLog.current_mission()

        if current == "explore":
            self.trigger_exploration(True)
        elif current == "Pathing":
            self.trigger_path(True)
        elif current == "visual_tune":
            self.send_arm_r_home_goal()
            self.trigger_visual_servo(True)
            self.get_logger().info("waiting on camera data visual_tune...")
        elif current == "hello_moveit":

            self.get_logger().info("Resuming at hello_moveit! Triggering arm sequence...")
            if not self.is_processing:
                self.is_processing = True
                if not self._arm_r_action_client.wait_for_server(timeout_sec=3.0):
                    self.get_logger().warn("Arm R controller action server not reached within 3s. Launching perception & MoveIt directly...")
                    self.launch_scripts()
                self.send_arm_r_home_goal()
        elif current is not None:
            val = self.missionLog.next_mission()
            if val and val != "":
                x, y, yaw = map(float, val.split(','))
                self.publish_goal(x, y, yaw)

    def trigger_exploration(self, enable: bool):
        self.exploration_publisher.publish(Bool(data=enable))
        self.is_exploring = enable
        self.get_logger().info(f"{'Started' if enable else 'Stopped'} exploration.")

    def trigger_path(self, enable: bool):
        self.path_enable = enable
        self.get_logger().info(f"{'Started' if enable else 'Stopped'} pathing.")

    def trigger_visual_servo(self, enable: bool):
        self.vs_trigger_publisher.publish(Bool(data=enable))
        self.is_visual_servoing = enable
        self.get_logger().info(f"{'Started' if enable else 'Stopped'} visual servoing.")

    def finished_callback(self, msg: Bool):
        if not msg.data:
            return

        current = self.missionLog.current_mission()

        if self.is_exploring:
            self.trigger_exploration(False)
            self.missionLog.check_done_mission()
        elif self.is_visual_servoing:
            self.trigger_visual_servo(False)
            self.missionLog.check_done_mission()
        elif current is not None:
            self.missionLog.check_done_mission()

        next_stage = self.missionLog.current_mission()
        if next_stage is None:
            self.get_logger().info("Mission done.")
            return

        # --- UPDATE THIS SECTION ---
        next_goal = self.missionLog.next_mission()
        if next_goal and next_goal != "":
            x, y, yaw = map(float, next_goal.split(','))
            self.publish_goal(x, y, yaw)
        elif next_stage == "visual_tune":
            self.get_logger().info("Switch to visual_tune. waiting on camera...")
        elif next_stage == "hello_moveit":
            self.get_logger().info("Switching to hello_moveit! Triggering arm sequence...")
            if not self.is_processing:
                self.is_processing = True
                self.send_arm_r_home_goal()

    def publish_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "warthog/base_link"
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_publisher.publish(goal_msg)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def path_feedback_callback(self, msg: Path):
        if not msg.poses:
            return

        current = self.missionLog.current_mission()
        if current != "Pathing":
            return

        if self.path_enable:
            if self.current_x is None or self.current_y is None:
                self.get_logger().warn("Waiting for odometry data to check path distance...")
                return

            last_pose = msg.poses[-1].pose.position
            distance = math.hypot(last_pose.x - self.current_x, last_pose.y - self.current_y)

            if distance < self.distance_tolerance:
                self.get_logger().info("Reached the last point! Canceling Nav2 mission...")

                self.trigger_path(False)

                if self.nav_cancel_client.wait_for_service(timeout_sec=1.0):
                    cancel_req = CancelGoal.Request()
                    self.nav_cancel_client.call_async(cancel_req)
                else:
                    self.get_logger().warn("Nav2 cancel service not available!")

                self.missionLog.check_done_mission()

                next_stage = self.missionLog.current_mission()
                if next_stage is None:
                    self.get_logger().info("All missions done.")
                    return

                next_goal = self.missionLog.next_mission()
                if next_goal and next_goal != "":
                    x, y, yaw = map(float, next_goal.split(','))
                    self.publish_goal(x, y, yaw)
                elif next_stage == "visual_tune":
                    self.get_logger().info("Switching to visual_tune. Waiting on camera...")

    def camera_feedback_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        current = self.missionLog.current_mission()
        if current != "visual_tune":
            return

        if not self.is_visual_servoing:
            self.get_logger().info("Stop exploring, start visual servo")
            if self.is_exploring:
                self.trigger_exploration(False)

            self.trigger_visual_servo(True)

    # Clean up subprocesses properly on shutdown
    def destroy_node(self):
        self.get_logger().info("Shutting down processes...")
        if self.process_moveit and self.process_moveit.poll() is None:
            self.process_moveit.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Abort mission control.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()