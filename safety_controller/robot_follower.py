import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np


class DynamicRobotFollowerNode(Node):
    def __init__(self):
        super().__init__('dynamic_robot_follower_node')

        # Parameters
        self.declare_parameter('distance', 1.0)  # Distance between robots
        self.distance = self.get_parameter(
            'distance').get_parameter_value().double_value

        # Initialize
        self.robot_publishers = {}
        self.robot_subscribers = {}
        self.robot_odometry = {}
        self.initial_world_odometry = {}  # Dictionary to store initial world odometry
        self.lead_robot = None

        # Discover robots
        self.discover_robots()

        # Subscribers
        for robot_name in self.robot_odometry.keys():
            self.create_subscription(Odometry, f'/{robot_name}/odom',
                                     lambda msg, name=robot_name: self.odom_callback(msg, name), QoSProfile(depth=10))

        self.get_logger().info(
            f'Discovered robots: {list(self.robot_publishers.keys())}')

    def discover_robots(self):
        topics = self.get_topic_names_and_types()

        for topic, types in topics:
            if topic.endswith('/cmd_vel') and 'geometry_msgs/msg/Twist' in types:

                robot_name = topic.split('/')[1]
                self.get_logger().info(
                    f'Found robot: {robot_name}')
                self.robot_publishers[robot_name] = self.create_publisher(
                    Twist, topic, QoSProfile(depth=10))
                # Initialize odometry data

                self.robot_subscribers[robot_name] = self.create_subscription(Odometry, f'/{robot_name}/odom',
                                                                              lambda msg, name=robot_name: self.odom_callback(msg, name), QoSProfile(depth=10))

        if not self.robot_publishers:
            self.get_logger().error('No robots discovered.')
            rclpy.sleep(rclpy.duration.Duration(seconds=2))
            self.discover_robots()

    def odom_callback(self, msg, robot_name):
        # Update the odometry for the specific robot
        self.robot_odometry[robot_name] = msg.pose.pose

        # If this is the first time we receive odometry data, save the initial odometry
        if robot_name not in self.initial_world_odometry:
            self.save_initial_odometry(robot_name, msg)

    def save_initial_odometry(self, robot_name, msg):
        """Saves the initial odometry for each discovered robot relative to the world frame."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        self.initial_world_odometry[robot_name] = {
            'position': {
                'x': position.x,
                'y': position.y,
                'z': position.z,
            },
            'orientation': {
                'roll': euler[0],
                'pitch': euler[1],
                'yaw': euler[2],
            }
        }
        self.get_logger().info(
            f"Initial odometry for {robot_name}: {self.initial_world_odometry[robot_name]}")

        # Determine the lead robot after initial odometry data is received
        if len(self.initial_world_odometry) == len(self.robot_publishers) and not self.lead_robot:
            self.determine_lead_robot()

    def determine_lead_robot(self):
        """Determines the lead robot based on initial positions along the y-axis."""
        if not self.initial_world_odometry:
            self.get_logger().error('Initial odometry data is empty.')
            return None

        # Extract the robot names and their corresponding y positions
        robots_x_positions = [(robot_name, data['position']['x'])
                              for robot_name, data in self.initial_world_odometry.items()]

        # Sort robots based on the y position
        robots_x_positions.sort(key=lambda item: item[1])
        # Find the middle robot
        middle_index = len(robots_x_positions) // 2

        self.lead_robot = robots_x_positions[middle_index][0]
        self.get_logger().info(f'Lead robot determined: {self.lead_robot}')
        return self.lead_robot

    def wait_for_message(self, msg_type, topic, timeout=10.0):
        """Waits for a single message from the specified topic."""
        msg = None
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot_odometry[topic.split('/')[1]] is not None:
                msg = self.robot_odometry[topic.split('/')[1]]
                break
        return msg

    def destroy_node(self):
        # Destroy all publishers and subscribers explicitly
        for publisher in self.robot_publishers.values():
            self.destroy_publisher(publisher)
        for subscriber in self.robot_subscribers.values():
            self.destroy_subscription(subscriber)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicRobotFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutdown requested via Ctrl+C')
    finally:
        if rclpy.ok():  # Only shutdown if rclpy is still running
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
