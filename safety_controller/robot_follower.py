import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
import math


class DynamicRobotFollowerNode(Node):
    def __init__(self):
        super().__init__('dynamic_robot_follower_node')
        # Initialize
        self.robot_publishers = {}
        self.robot_subscribers = {}
        self.robot_odometry = {}
        self.initial_world_odometry = {}  # Dictionary to store initial world odometry
        self.lead_robot = None
        self.aligning_robots = set()
        self.target_positions = {}

        # Discover robots
        self.discover_robots()
        self.lead_timer = self.create_timer(0.5, self.check_lead_robot)

    def check_lead_robot(self):
        if self.lead_robot is None:
            self.get_logger().info("Waiting for lead robot to be determined...")
        else:
            self.get_logger().info(f"Lead robot determined timer stopping")
            self.lead_timer.cancel()
            self.initialize_target_positions()
            self.create_timer(0.5, self.check_robot_position)

    def check_robot_position(self):
        for i, robot_name in enumerate(self.robot_publishers):
            if robot_name != self.lead_robot:
                self.navigate_to_target(robot_name)

    def initialize_target_positions(self):
        """Initializes target positions for each robot relative to the lead robot."""
        lead_index = list(self.robot_publishers.keys()).index(self.lead_robot)

        for i, robot_name in enumerate(self.robot_publishers):
            # Distance of 0.5 meters apart in x
            relative_position_x = (i - lead_index) * 0.5
            relative_position_y = 0.0  # All robots should have y=0 in target position
            relative_position_z = 0.0  # All robots should have z=0 in target position
            # No rotation relative to the lead robot
            target_orientation = [0.0, 0.0, 0.0, 1.0]

            # Create a new Odometry message for the target position
            target_odom = Odometry()

            # Set the target position
            target_odom.pose.pose.position.x = relative_position_x
            target_odom.pose.pose.position.y = relative_position_y
            target_odom.pose.pose.position.z = relative_position_z

            # Set the target orientation
            target_odom.pose.pose.orientation.x = target_orientation[0]
            target_odom.pose.pose.orientation.y = target_orientation[1]
            target_odom.pose.pose.orientation.z = target_orientation[2]
            target_odom.pose.pose.orientation.w = target_orientation[3]

            # Set the header information
            target_odom.header.stamp = self.get_clock().now().to_msg()
            target_odom.header.frame_id = 'relative_to_' + self.lead_robot

            # Store the target odometry in the dictionary
            self.target_positions[robot_name] = target_odom

            self.get_logger().info(
                f"Target position for {robot_name}: x={relative_position_x}, y={relative_position_y}, z={relative_position_z}")

        return self.target_positions

    def print_relative(self):
        for robot_name in self.robot_publishers.keys():
            if robot_name != self.lead_robot:
                print(robot_name)
                relative_odom = self.get_relative_odometry(robot_name)
                if relative_odom:
                    relative_position = relative_odom.pose.pose.position
                    self.get_logger().info(
                        f"Relative position of {robot_name} to {self.lead_robot}: x={relative_position.x}, y={relative_position.y}, z={relative_position.z}")

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
            time.sleep(5)
            self.discover_robots()
            return

        self.get_logger().info(
            f'Discovered robots: {list(self.robot_publishers.keys())}')

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

        # Reorder the robot publishers and subscribers based on x positions
        sorted_publishers = {name: self.robot_publishers[name]
                             for name, _ in robots_x_positions if name in self.robot_publishers}
        sorted_subscribers = {name: self.robot_subscribers[name]
                              for name, _ in robots_x_positions if name in self.robot_subscribers}

        self.robot_publishers = sorted_publishers
        self.robot_subscribers = sorted_subscribers

        self.get_logger().info(f'Lead robot determined: {self.lead_robot}')
        self.get_logger().info(
            f'Robot publishers ordered: {list(self.robot_publishers.keys())}')
        self.get_logger().info(
            f'Robot subscribers ordered: {list(self.robot_subscribers.keys())}')

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

    def get_relative_odometry(self, robot_name):
        """Returns the odometry of the given robot relative to the lead robot."""
        if self.lead_robot is None:
            self.get_logger().error("Lead robot has not been determined yet.")
            return None
        elif robot_name not in self.robot_odometry:
            self.get_logger().error(f"Robot {robot_name} not found.")
            return None
        elif self.robot_odometry[robot_name] is None or self.robot_odometry[self.lead_robot] is None:
            self.get_logger().error(
                f"Odometry data for {robot_name} or lead robot is missing.")
            return None

        # Get the absolute positions and orientations (directly from Pose object)
        robot_pose = self.robot_odometry[robot_name]
        lead_pose = self.robot_odometry[self.lead_robot]

        # Compute relative position
        relative_position = [
            robot_pose.position.x - lead_pose.position.x,
            robot_pose.position.y - lead_pose.position.y,
            robot_pose.position.z - lead_pose.position.z,
        ]

        # Compute relative orientation using quaternion multiplication
        # Relative orientation = robot_orientation * inverse(lead_orientation)
        lead_quaternion = [
            lead_pose.orientation.x,
            lead_pose.orientation.y,
            lead_pose.orientation.z,
            lead_pose.orientation.w,
        ]
        robot_quaternion = [
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        ]

        lead_quaternion_inv = tf_transformations.quaternion_inverse(
            lead_quaternion)
        relative_quaternion = tf_transformations.quaternion_multiply(
            robot_quaternion, lead_quaternion_inv)

        # Create a new Odometry message for the relative odometry
        relative_odom = Odometry()

        # Set the relative position
        relative_odom.pose.pose.position.x = relative_position[0]
        relative_odom.pose.pose.position.y = relative_position[1]
        relative_odom.pose.pose.position.z = relative_position[2]

        # Set the relative orientation
        relative_odom.pose.pose.orientation.x = relative_quaternion[0]
        relative_odom.pose.pose.orientation.y = relative_quaternion[1]
        relative_odom.pose.pose.orientation.z = relative_quaternion[2]
        relative_odom.pose.pose.orientation.w = relative_quaternion[3]

        # Set the header information
        relative_odom.header.stamp = self.get_clock().now().to_msg()
        relative_odom.header.frame_id = 'relative_to_' + self.lead_robot

        self.get_logger().info(
            f"Relative odometry for {robot_name} relative to {self.lead_robot} calculated.")

        return relative_odom

    def navigate_to_target(self, robot_name, alignment_threshold=0.12):
        """Check the current position of the robot and navigate it to the target position if misaligned."""

        if robot_name not in self.target_positions:
            self.get_logger().error(
                f"Target position for {robot_name} not found.")
            return
        elif self.robot_odometry.get(robot_name) is None:
            self.get_logger().error(
                f"Odometry data for {robot_name} is missing.")
            return

        # Get current position and target position
        current_pos = self.get_relative_odometry(robot_name).pose.pose.position
        target_pos = self.target_positions[robot_name].pose.pose.position

        # Calculate current relative position
        current_pos = [current_pos.x, current_pos.y]
        target_pos = [target_pos.x, target_pos.y]

        distance = self.calculate_distance(current_pos, target_pos)

        # Check if the robot is within the threshold
        if distance > alignment_threshold and not (robot_name in self.aligning_robots):
            self.aligning_robots.add(robot_name)
            self.send_navigation_command(robot_name, current_pos, target_pos)
        else:
            # Stop the robot if it's within the threshold
            stop_cmd = Twist()
            if robot_name in self.robot_publishers:
                self.robot_publishers[robot_name].publish(stop_cmd)
                self.get_logger().info(
                    f"{robot_name} is aligned with target. Stopping.")
            else:
                self.get_logger().error(
                    f"No publisher found for {robot_name}.")
        if robot_name in self.aligning_robots:
            self.aligning_robots.remove(robot_name)

    def get_yaw_from_quaternion(self, orientation):
        """Convert quaternion to yaw angle."""
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Compute yaw (Z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def send_navigation_command(self, robot_name, current_pos, target_pos):
        # Get the current relative position of the robot and its target position
        current_relative_odom = self.get_relative_odometry(robot_name)

        # Calculate the distance and angle to the target
        distance = self.calculate_distance(current_pos, target_pos)
        target_angle = math.atan2(
            target_pos[1] - current_pos[1], target_pos[0] - current_pos[0])

        # Get the robot's current orientation in radians
        current_orientation = current_relative_odom.pose.pose.orientation
        current_angle = tf_transformations.euler_from_quaternion(
            [current_orientation.x, current_orientation.y,
                current_orientation.z, current_orientation.w]
        )[2]

        # Compute the angular difference
        angle_diff = target_angle - current_angle

        # Normalize the angle difference to be within -pi to pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Define proportional gains
        K_linear = 1.0  # Linear velocity gain
        K_angular = 4.0  # Angular velocity gain

        # Set linear and angular velocity based on distance and angle
        cmd = Twist()
        cmd.linear.x = K_linear * distance   # Scale linear velocity
        scaling_factor = min(0.1 / cmd.linear.x, 1)
        cmd.linear.x = cmd.linear.x * scaling_factor
        # Angular velocity proportional to angle difference
        cmd.angular.z = K_angular * angle_diff * scaling_factor

        # Publish command
        if robot_name in self.robot_publishers:
            self.robot_publishers[robot_name].publish(cmd)
            self.get_logger().info(
                f"Sending command to {robot_name}: Linear={cmd.linear.x}, Angular={cmd.angular.z}")
        else:
            self.get_logger().error(
                f"No publisher found for robot {robot_name}")


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
