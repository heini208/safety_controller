### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
import smach

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from rclpy.node import Node
from std_msgs.msg import Float32


# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery', 'collision_detected', 'idle'])
        self.node = node
        self.battery_voltage = 100.0
        self.collision_detected = False
        self.collision_threshold = 0.5
        self.battery_threshold = 20.0
        self.node.create_subscription(Float32, 'battery_voltage', self.battery_callback, 10)
        self.node.create_subscription(LaserScan, 'scan', self.collision_callback, 10)


    def battery_callback(self, msg):
        self.battery_voltage = msg.data
        
    def collision_callback(self, msg):
        if min(msg.ranges) < self.collision_threshold:
            self.collision_detected = True
        else:
            self.collision_detected = False

    def execute(self, userdata):
        rclpy.spin_once(self.node)
        if self.battery_voltage < self.battery_threshold:
            return 'low_battery'
        elif self.collision_detected:
            return 'collision_detected'
        else:
            return 'idle'



class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['battery_ok'])
        self.node = node
        self.battery_voltage = 100.0
        self.rotation_speed = 0.5
        self.battery_threshold = 20.0

        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.node.create_subscription(Float32, 'battery_voltage', self.battery_callback, 10)

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def execute(self, userdata):
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.publisher.publish(twist)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.battery_voltage >= self.battery_threshold:
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                return 'battery_ok'
            time.sleep(0.1)


class StopMotion(smach.State):
    """State to stop the robot's motion
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['collision_avoided'])
        self.node = node
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)


    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

        while True:
            time.sleep(1)
            return 'collision_avoided'


def main(args=None):
    """Main function to initialise and execute the state machine
    """
    rclpy.init(args=args)
    node = Node('robile_state_machine')

    state_machine = smach.StateMachine(outcomes=[])

    with state_machine:
        smach.StateMachine.add('MONITOR', MonitorBatteryAndCollision(node),
                               transitions={'low_battery': 'ROTATE',
                                            'collision_detected': 'STOP',
                                            'idle': 'MONITOR'})

        smach.StateMachine.add('ROTATE', RotateBase(node),
                               transitions={'battery_ok': 'MONITOR'})

        smach.StateMachine.add('STOP', StopMotion(node),
                               transitions={'collision_avoided': 'MONITOR'})

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()