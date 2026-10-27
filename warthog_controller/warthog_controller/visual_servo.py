import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class VisualServoController(Node):
    def __init__(self):
        super().__init__('visual_servo_controller')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parametrar
        self.declare_parameter('cmd_vel_topic', 'warthog/cmd_vel_step')
        self.declare_parameter('camera_topic', 'warthog/poses')
        self.declare_parameter('reached_topic', 'warthog/reached')
        self.declare_parameter("visual_servo_trigger", "warthog/visual_servo_trigger")
        self.declare_parameter('kp_linear', 0.7)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('standoff_distance', 1.40)
        self.declare_parameter('dist_tolerance', 0.2)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.2)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        camera_topic = self.get_parameter('camera_topic').value
        
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.standoff_distance = float(self.get_parameter('standoff_distance').value)
        self.dist_tolerance = float(self.get_parameter('dist_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        reached_topic = self.get_parameter('reached_topic').value
        vs_trigger_topic = self.get_parameter("visual_servo_trigger").value

        self.stop_counter = 0
        self.stop_bound = 5
        self.stoped = False
        self.target_frame = 'warthog/base_link'
        self.active = False

        # Publishers & Subscribers
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.camera_sub = self.create_subscription(PoseArray, camera_topic, self.camera_callback, 10)
        self.vs_trigger_sub = self.create_subscription(Bool, vs_trigger_topic, self.vs_callback, 10)
        self.reached_publisher = self.create_publisher(Bool, reached_topic, 10)

        self.get_logger().info("Visual Servo Controller started.")
    def vs_callback(self,msg):
        if not msg:
            return
        if msg is True:
            self.active = True
        if msg is False:
            self.active = False
    def camera_callback(self, msg: PoseArray):
        #if self.active is False:
         #   return
        if self.stop_counter > self.stop_bound:
            if self.stoped is False:
                self.stop_robot()
                self.stoped = True
                print(f'stoped the robot')
                self.goal_reached = True
                #self.get_logger().info("Target position and orientation reached!")
                reached_msg = Bool()
                reached_msg.data = True
                self.reached_publisher.publish(reached_msg)
            return
        if not msg.poses:
        
            self.stop_robot()
            return
        
        pose_to_transform = PoseStamped()
        pose_to_transform.header = msg.header
        pose_to_transform.pose = msg.poses[0]

        try:
            # 2. Transform the PoseStamped
            transformed_pose = self.tf_buffer.transform(
                pose_to_transform,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f'Could not transform: {ex}')
            return

        # 3. Access the actual pose data inside the PoseStamped
        detected_pose = transformed_pose.pose
        
        # Use the original message's frame_id to check if it was an optical frame
        frame_id = msg.header.frame_id.lower()

        print(detected_pose.position.x, detected_pose.position.y)

        print(frame_id)
        if "optical" in frame_id:
            rel_forward = detected_pose.position.z
            rel_lateral = -detected_pose.position.x
        else:
            rel_forward = detected_pose.position.x
            rel_lateral = detected_pose.position.y
        rel_forward = detected_pose.position.x
        rel_lateral = detected_pose.position.y

        print(f'forward {rel_forward}, lateral {rel_lateral}')
        dist_to_obj = math.hypot(rel_forward, rel_lateral)
        dist_error = dist_to_obj - self.standoff_distance
        print(f'dist error {dist_error}')
        
       
        target_angle = math.atan2(rel_lateral, rel_forward)

        x_speed = 0.0
        z_speed = 0.0

        
        if abs(dist_error) > self.dist_tolerance:
            x_speed = self.kp_linear * dist_error
            z_speed = self.kp_angular * target_angle
            
          
            x_speed = np.clip(x_speed, -self.max_linear_speed, self.max_linear_speed)
            z_speed = np.clip(z_speed, -self.max_angular_speed, self.max_angular_speed)
        else:
            self.stop_counter += 1

        # print(f'X_speed {x_speed}, z_speed {z_speed}')
        self.publish_cmd(x_speed, z_speed)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, x, z):
        twist_msg = Twist()
        twist_msg.linear.x = float(x)
        twist_msg.angular.z = float(z)
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Abort visuell servoing.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()