import time
import unittest
import launch
import launch_ros
import launch_testing
import rclpy
import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from safety_controller.robot_follower import DynamicRobotFollowerNode
from unittest import mock


@pytest.mark.rostest
def generate_test_description():
    follower_node = launch_ros.actions.Node(
        package='safety_controller',
        executable='follower_node',
        name='follower_node',
        output='log',
        parameters=[],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return launch.LaunchDescription([
        follower_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'follower_node': follower_node}


class TestDynamicRobotFollowerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = DynamicRobotFollowerNode()
        self.node.robot_publishers = {
            'robot_1': mock.Mock(),
            'robot_2': mock.Mock(),
        }
        self.node.robot_subscribers = {
            'robot_1': mock.Mock(),
            'robot_2': mock.Mock(),
        }
        self.received_odom_msgs = {}
        self.cmd_vel_msgs = {}

    def tearDown(self):
        self.node.destroy_node()

    def odom_callback(self, msg, robot_name):
        self.received_odom_msgs[robot_name] = msg

    def cmd_vel_callback(self, msg, robot_name):
        self.cmd_vel_msgs[robot_name] = msg

    def test_robot_discovery(self, follower_node, proc_output):
        # Subscribe to /robot_1/odom and /robot_2/odom topics
        robots = ['robot_1', 'robot_2']
        for robot in robots:
            self.node.create_subscription(
                Odometry, f'/{robot}/odom', lambda msg, r=robot: self.odom_callback(msg, r), 10)
            self.node.create_subscription(
                Twist, f'/{robot}/cmd_vel', lambda msg, r=robot: self.cmd_vel_callback(msg, r), 10)

        # Give time for the node to discover robots and start publishing
        self.node.get_logger().info('Waiting for robot discovery...')
        time.sleep(5)

        # Check if odometry messages are received, which implies robots were discovered
        for robot in robots:
            self.assertIn(robot, self.received_odom_msgs,
                          f"{robot} was not discovered.")

    def test_lead_robot_determination(self):
        self.node.initial_world_odometry = {
            'robot_1': {'position': {'x': 1.0}},
            'robot_2': {'position': {'x': 2.0}},
            'robot_3': {'position': {'x': 3.0}},
        }

        lead_robot = self.node.determine_lead_robot()
        self.assertIsNotNone(lead_robot, "Lead robot should not be None.")
        self.assertEqual(lead_robot, 'robot_2',
                         "The lead robot should be 'robot_2' as it is in the middle.")

    def test_relative_odometry_calculation(self, follower_node, proc_output):
        # Publish some test odometry messages
        robots = ['robot_1', 'robot_2']
        for robot in robots:
            test_odom = Odometry()
            test_odom.pose.pose.position.x = 1.0 if robot == 'robot_1' else 2.0
            test_odom.pose.pose.position.y = 0.0
            test_odom.pose.pose.position.z = 0.0
            self.received_odom_msgs[robot] = test_odom

        # Give some time for the node to process
        time.sleep(2)

        # Test relative odometry calculation
        rel_odom = self.node.get_relative_odometry('robot_2')
        self.assertIsNotNone(rel_odom, "Relative odometry was not calculated.")
        self.assertAlmostEqual(rel_odom.pose.pose.position.x, 1.0,
                               places=2, msg="Relative odometry x position is incorrect.")

    def test_navigate_to_target(self, follower_node, proc_output):
        # Assume target positions are initialized and check navigation command
        robots = ['robot_1', 'robot_2']
        for robot in robots:
            self.node.navigate_to_target(robot)
            time.sleep(1)

            self.assertIn(robot, self.cmd_vel_msgs,
                          f"No cmd_vel message received for {robot}.")
            # Check if the robot is moving towards the target
            cmd_vel = self.cmd_vel_msgs[robot]
            self.assertNotEqual(cmd_vel.linear.x, 0,
                                f"{robot} did not move towards target.")

    def test_stop_robot(self, follower_node, proc_output):
        # Issue a command to stop the robot
        robot = 'robot_1'
        self.node.stop_robot(robot)
        time.sleep(1)

        self.assertIn(robot, self.cmd_vel_msgs,
                      f"No cmd_vel message received for {robot}.")
        cmd_vel = self.cmd_vel_msgs[robot]
        self.assertEqual(cmd_vel.linear.x, 0,
                         f"{robot} did not stop as expected.")
        self.assertEqual(cmd_vel.angular.z, 0,
                         f"{robot} did not stop rotating as expected.")


if __name__ == '__main__':
    import launch_testing.main
    launch_testing.main()
