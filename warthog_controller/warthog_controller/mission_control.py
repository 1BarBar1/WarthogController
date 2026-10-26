import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, PoseStamped
from typing import Optional
import math

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

        # Parametrar
        self.declare_parameter("exploration_trigger", "warthog/exploration_trigger")
        self.declare_parameter("visual_servo_trigger", "warthog/visual_servo_trigger")
        self.declare_parameter("reached_topic", "warthog/reached")
        self.declare_parameter("goal_topic", "warthog/goal_pose")
        self.declare_parameter("camera_feedback", "warthog/poses")
        self.declare_parameter("auto_start", True)

        missions = {"visual_tune": ""}
        self.missionLog = MissionLog(missions)

        explore_topic = self.get_parameter("exploration_trigger").value
        vs_trigger_topic = self.get_parameter("visual_servo_trigger").value
        reached_topic = self.get_parameter("reached_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.camera_topic = self.get_parameter("camera_feedback").value
        self.auto_start = self.get_parameter("auto_start").value

        # Publishers
        self.exploration_publisher = self.create_publisher(Bool, explore_topic, 10)
        self.vs_trigger_publisher = self.create_publisher(Bool, vs_trigger_topic, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)

        # Subscribers
        self.finished_subscription = self.create_subscription(Bool, reached_topic, self.finished_callback, 10)
        self.camera_subscription = self.create_subscription(PoseArray, self.camera_topic, self.camera_feedback_callback, 10)

        self.is_exploring = False
        self.is_visual_servoing = False

        self.get_logger().info("Mission Control started.")

        if self.auto_start:
            self.start_timer = self.create_timer(1.0, self.start_robot)

    def start_robot(self):
        self.start_timer.cancel()
        current = self.missionLog.current_mission()
        if current == "explore":
            self.trigger_exploration(True)
        elif current == "visual_tune":
            self.get_logger().info("waitin on camera data visual_tune...")
        elif current is not None:
            val = self.missionLog.next_mission()
            if val and val != "":
                x, y, yaw = map(float, val.split(','))
                self.publish_goal(x, y, yaw)

    def trigger_exploration(self, enable: bool):
        self.exploration_publisher.publish(Bool(data=enable))
        self.is_exploring = enable
        self.get_logger().info(f"{'Started' if enable else 'Stoped'} exploration.")

    def trigger_visual_servo(self, enable: bool):
        self.vs_trigger_publisher.publish(Bool(data=enable))
        self.is_visual_servoing = enable
        self.get_logger().info(f"{'Started' if enable else 'Stoped'} visuell servoing.")

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
            self.get_logger().info("mission dones")
            return

        next_goal = self.missionLog.next_mission()
        if next_goal and next_goal != "":
            x, y, yaw = map(float, next_goal.split(','))
            self.publish_goal(x, y, yaw)
        elif next_stage == "visual_tune":
            self.get_logger().info("Switch to visual_tune. waiting on camera...")

    def publish_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "warthog/base_link"
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_publisher.publish(goal_msg)

    def camera_feedback_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        current = self.missionLog.current_mission()
        if current != "visual_tune":
            return

       
        if not self.is_visual_servoing:
            self.get_logger().info("stop exploring visual servo")
            if self.is_exploring:
                self.trigger_exploration(False)
            
            self.trigger_visual_servo(True)

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