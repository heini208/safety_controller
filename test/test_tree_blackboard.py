import time
import unittest
import launch
import launch_ros
import launch_testing
import rclpy
import pytest
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import py_trees as pt


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


class TestBlackboardVariables(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_blackboard_variables')
        self.bb = pt.blackboard.Blackboard()
        self.battery_low_warning_sub = self.node.create_subscription(
            Float32, 'battery_voltage', self.battery_callback, 10)
        self.collision_warning_sub = self.node.create_subscription(
            LaserScan, 'scan', self.collision_callback, 10)

    def tearDown(self):
        self.node.destroy_node()

    def battery_callback(self, msg):
        self.bb.battery_low_warning = msg.data < 11.0

    def collision_callback(self, msg):
        self.bb.collision_warning = any(dist < 0.2 for dist in msg.ranges)

    def test_battery_low_warning(self):
        pub_battery = self.node.create_publisher(
            Float32, 'battery_voltage', 10)
        msg = Float32()
        msg.data = 10.0
        pub_battery.publish(msg)

        time.sleep(2)
        rclpy.spin_once(self.node, timeout_sec=2)

        self.assertTrue(hasattr(self.bb, 'battery_low_warning') and self.bb.battery_low_warning,
                        "Battery low warning not set correctly on blackboard")

    def test_collision_warning(self):
        pub_scan = self.node.create_publisher(LaserScan, 'scan', 10)
        scan = LaserScan()
        scan.ranges = [0.1] * 360
        pub_scan.publish(scan)

        time.sleep(2)
        rclpy.spin_once(self.node, timeout_sec=2)

        self.assertTrue(hasattr(self.bb, 'collision_warning') and self.bb.collision_warning,
                        "Collision warning not set correctly on blackboard")


if __name__ == '__main__':
    launch_testing.main()
