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
from unittest.mock import MagicMock
import threading


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
        # Initialize the DynamicRobotFollowerNode
        # Create mock publishers and subscribers
        self.mock_publishers = {
            'robot_1': MagicMock(),
            'robot_2': MagicMock(),
        }
        self.mock_subscribers = {
            'robot_1': MagicMock(),
            'robot_2': MagicMock(),
        }
        self.received_odom_msgs = {}
        self.cmd_vel_msgs = {}

        self.node = DynamicRobotFollowerNode()
        # Setup mock subscriptions for received messages
        for robot in ['robot_1', 'robot_2']:
            self.node.create_subscription(
                Odometry, f'/{robot}/odom', lambda msg, r=robot: self.odom_callback(msg, r), 10)
            self.node.create_subscription(
                Twist, f'/{robot}/cmd_vel', lambda msg, r=robot: self.cmd_vel_callback(msg, r), 10)

        self.node.robot_publishers = self.mock_publishers
        self.node.robot_subscribers = self.mock_subscribers

        # Start the node in a separate thread to process callbacks
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.executor_thread.join()
        self.node.destroy_node()

    def odom_callback(self, msg, robot_name):
        self.received_odom_msgs[robot_name] = msg

    def cmd_vel_callback(self, msg, robot_name):
        self.cmd_vel_msgs[robot_name] = msg
        self.node.get_logger().info(f'Received cmd_vel message: {msg}')

    def test_robot_discovery(self, follower_node, proc_output):
        # Simulate publishing odometry messages
        for robot in ['robot_1', 'robot_2']:
            test_odom = Odometry()
            test_odom.pose.pose.position.x = 1.0 if robot == 'robot_1' else 2.0
            test_odom.pose.pose.position.y = 0.0
            test_odom.pose.pose.position.z = 0.0
            self.mock_publishers[robot].publish(test_odom)

        # Give time for the node to process
        self.node.discover_robots()
        time.sleep(2)

        # Check if odometry messages are received
        for robot in ['robot_1', 'robot_2']:
            self.assertIn(robot, self.node.robot_publishers,
                          f"{robot} was not discovered.")

    def test_lead_robot_determination(self):
        # Provide initial odometry data
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
        # Create test odometry messages
        odom_msg_1 = Odometry()
        odom_msg_1.pose.pose.position.x = 1.0
        odom_msg_1.pose.pose.position.y = 0.0
        odom_msg_1.pose.pose.position.z = 0.0

        odom_msg_2 = Odometry()
        odom_msg_2.pose.pose.position.x = 2.0
        odom_msg_2.pose.pose.position.y = 0.0
        odom_msg_2.pose.pose.position.z = 0.0

        self.node.robot_odometry['robot_1'] = odom_msg_1.pose.pose
        self.node.robot_odometry['robot_2'] = odom_msg_2.pose.pose

        self.node.lead_robot = 'robot_2'

        time.sleep(0.5)

        rel_odom = self.node.get_relative_odometry('robot_1')
        self.assertIsNotNone(rel_odom, "Relative odometry was not calculated.")
        self.assertAlmostEqual(rel_odom.pose.pose.position.x, -1.0,
                               places=2, msg="Relative odometry x position is incorrect")

    def test_navigate_to_target(self, follower_node, proc_output):
        odom = Odometry()
        odom.pose.pose.position.x = 10.0
        odom.pose.pose.position.y = 10.0
        odom.pose.pose.position.z = 0.0
        # Store the Odometry message in target_positions
        self.node.target_positions['robot_1'] = odom

        self.node.lead_robot = 'robot_2'
        odom_msg_1 = Odometry()
        odom_msg_1.pose.pose.position.x = 1.0
        odom_msg_1.pose.pose.position.y = 1.0
        odom_msg_1.pose.pose.position.z = 0.0

        odom_msg_2 = Odometry()
        odom_msg_2.pose.pose.position.x = 2.0
        odom_msg_2.pose.pose.position.y = 2.0
        odom_msg_2.pose.pose.position.z = 0.0

        self.node.robot_odometry['robot_1'] = odom_msg_1.pose.pose
        self.node.robot_odometry['robot_2'] = odom_msg_2.pose.pose
        self.node.navigate_to_target('robot_1')
        time.sleep(1)

        self.mock_publishers['robot_1'].publish.assert_called()
        published_cmd_vel = self.mock_publishers['robot_1'].publish.call_args[0][0]
        self.assertIsInstance(published_cmd_vel, Twist,
                              "Published message is not of type Twist.")

    def test_stop_robot(self, follower_node, proc_output):
        robot = 'robot_1'
        self.node.stop_robot(robot)

        self.mock_publishers['robot_1'].publish.assert_called()
        published_cmd_vel = self.mock_publishers['robot_1'].publish.call_args[0][0]
        self.assertIsInstance(published_cmd_vel, Twist,
                              "Published message is not of type Twist.")
        self.assertEqual(published_cmd_vel.linear.x, 0,
                         "robot_1 did is still moving target.")
