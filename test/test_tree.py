import os
import sys
import time
import unittest
import launch
import launch_ros
import launch_testing
import rclpy
import pytest
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

@pytest.mark.rostest
def generate_test_description():
    tree_node = launch_ros.actions.Node(
        package='safety_controller',
        executable='tree_node',
        name='tree_node',
        output='log',  
        parameters=[],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return launch.LaunchDescription([
        tree_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'tree_node': tree_node}


class TestLowBatteryRotation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_behavior_tree_listener')
        self.cmd_vel_msgs = []

    def tearDown(self):
        self.node.destroy_node()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msgs.append(msg)
        self.node.get_logger().info(f'Received cmd_vel message: {msg}')

    def test_battery_status(self, tree_node, proc_output):
        subscription_cmd_vel = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        pub_battery = self.node.create_publisher(
            Float32,
            'battery_voltage',
            10
        )

        try:
            self.node.get_logger().info('Waiting for nodes to set up...')
            time.sleep(3)

            self.node.get_logger().info(f'Available topics: {self.node.get_topic_names_and_types()}')

            # Publish a low battery voltage to trigger rotation
            msg = Float32()
            msg.data = 0.0
            self.node.get_logger().info('Publishing low battery voltage...')
            pub_battery.publish(msg)
            time.sleep(5)

            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if len(self.cmd_vel_msgs) > 2:
                    break

            self.assertGreater(len(self.cmd_vel_msgs), 2, "No cmd_vel messages received, behavior tree might not be running")

            for msg in self.cmd_vel_msgs:
                self.assertIsInstance(msg, Twist, "Received message is not of type Twist")
                self.assertNotEqual(msg.angular.z, 0, "Robot is not rotating")


        finally:
            self.node.destroy_subscription(subscription_cmd_vel)
            self.node.destroy_publisher(pub_battery)

    def test_collision_avoidance_stop(self, tree_node, proc_output):
        subscription_cmd_vel = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        pub_scan = self.node.create_publisher(
            LaserScan,
            'scan',
            10
        )

        try:
            # Publish an initial cmd_vel message
            initial_cmd_vel = Twist()
            initial_cmd_vel.linear.x = 0.5
            initial_cmd_vel.angular.z = 0.0
            self.node.get_logger().info('Publishing initial cmd_vel message...')
            self.node.get_logger().info('Waiting for nodes to set up...')
            time.sleep(3)
            self.cmd_vel_msgs.append(initial_cmd_vel)

            # Publish a LaserScan message to simulate an obstacle
            scan_msg = LaserScan()
            scan_msg.ranges = [0.1] * 360  # Obstacle very close
            self.node.get_logger().info('Publishing LaserScan message with close obstacle...')
            pub_scan.publish(scan_msg)
            time.sleep(5)

            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if any(msg.linear.x == 0 and msg.angular.z == 0 for msg in self.cmd_vel_msgs):
                    break

            self.assertTrue(any(msg.linear.x == 0 and msg.angular.z == 0 for msg in self.cmd_vel_msgs), "Robot did not stop despite close obstacle")

        finally:
            self.node.destroy_subscription(subscription_cmd_vel)
            self.node.destroy_publisher(pub_scan)

    def test_collision_avoidance_no_stop(self, tree_node, proc_output):
        subscription_cmd_vel = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        pub_scan = self.node.create_publisher(
            LaserScan,
            'scan',
            10
        )

        try:
            # Publish an initial cmd_vel message
            initial_cmd_vel = Twist()
            initial_cmd_vel.linear.x = 0.5
            initial_cmd_vel.angular.z = 0.0
            self.node.get_logger().info('Publishing initial cmd_vel message...')
            self.node.get_logger().info('Waiting for nodes to set up...')
            time.sleep(3)
            self.cmd_vel_msgs.append(initial_cmd_vel)

            # Publish a LaserScan message to simulate no obstacle
            scan_msg = LaserScan()
            scan_msg.ranges = [5.0] * 360  # No obstacles nearby
            self.node.get_logger().info('Publishing LaserScan message with no obstacle...')
            pub_scan.publish(scan_msg)
            time.sleep(5)

            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)

            self.assertTrue(all(msg.linear.x != 0 or msg.angular.z != 0 for msg in self.cmd_vel_msgs), "Robot stopped despite no obstacles")

        finally:
            self.node.destroy_subscription(subscription_cmd_vel)
            self.node.destroy_publisher(pub_scan)

    def test_collision_avoidance_recovery(self, tree_node, proc_output):
        subscription_cmd_vel = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        pub_scan = self.node.create_publisher(
            LaserScan,
            'scan',
            10
        )

        try:
            initial_cmd_vel = Twist()
            initial_cmd_vel.linear.x = 0.5
            initial_cmd_vel.angular.z = 0.0
            self.node.get_logger().info('Publishing initial cmd_vel message...')
            self.node.get_logger().info('Waiting for nodes to set up...')
            time.sleep(3)
            self.cmd_vel_msgs.append(initial_cmd_vel)

            scan_msg = LaserScan()
            scan_msg.ranges = [0.1] * 360  # Obstacle very close
            self.node.get_logger().info('Publishing LaserScan message with close obstacle...')
            pub_scan.publish(scan_msg)
            time.sleep(5)

            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if any(msg.linear.x == 0 and msg.angular.z == 0 for msg in self.cmd_vel_msgs):
                    break

            self.assertTrue(any(msg.linear.x == 0 and msg.angular.z == 0 for msg in self.cmd_vel_msgs), "Robot did not stop despite close obstacle")

            scan_msg = LaserScan()
            scan_msg.ranges = [5.0] * 360  # No obstacles nearby
            self.node.get_logger().info('Publishing LaserScan message with no obstacle...')
            pub_scan.publish(scan_msg)
            time.sleep(5)

            # Publish a new cmd_vel message to simulate resuming motion
            resume_cmd_vel = Twist()
            resume_cmd_vel.linear.x = 0.5
            resume_cmd_vel.angular.z = 0.0
            self.node.get_logger().info('Publishing cmd_vel message to resume motion...')
            self.cmd_vel_msgs.append(resume_cmd_vel)

            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)

            # Assert that the robot resumed motion
            self.assertTrue(any(msg.linear.x != 0 or msg.angular.z != 0 for msg in self.cmd_vel_msgs), "Robot did not resume motion despite obstacle removal")

        finally:
            self.node.destroy_subscription(subscription_cmd_vel)
            self.node.destroy_publisher(pub_scan)

if __name__ == '__main__':
    import launch_testing.main
    launch_testing.main()

