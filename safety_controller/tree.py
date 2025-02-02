# Implement the safety functionalities for the Robile by implementing all
# required behaviours here. Feel free to define additional behaviours if necessary

import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import py_trees.console as console
import sys


class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """

    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        super(Rotate, self).__init__(name)
        self.topic_name = topic_name
        self.rotation_speed = ang_vel

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behaviour")
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}]".format(
                self.qualified_name)
            raise KeyError(error_message) from e

        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("[ROTATE] update: updating rotate behaviour")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        twist_msg = Twist()
        twist_msg.angular.z = self.rotation_speed
        self.publisher.publish(twist_msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info(
            "[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0

        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)


class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """

    def __init__(self, name="stop platform", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        self.logger.info("[STOP] initialising stopping behavior")
        self.topic_name = topic_name

    def setup(self, **kwargs):
        self.logger.info("[STOP MOTION] setting up stop motion behaviour")
        self.node = kwargs['node']
        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)

        return True

    def update(self):
        self.logger.info("[STOP MOTION] update: stopping motion")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        return pt.common.Status.SUCCESS


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status
    """

    def __init__(self, battery_voltage_topic_name: str = "/battery_voltage",
                 name: str = 'Battery2BB',
                 threshold: float = 30.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         # to decide when data should be cleared/reset.
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        self.blackboard.register_key(
            key='battery_low_warning', access=pt.common.Access.WRITE)

        self.blackboard.battery_low_warning = False
        self.threshold = threshold

    def update(self):
        """
        check battery voltage level stored in self.blackboard.battery. By comparing with 
        threshold value, update the value of self.blackboad.battery_low_warning
        """
        self.logger.info('[BATTERY] update: running battery_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)

        super(BatteryStatus2bb, self).update()
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning = True
        else:
            self.blackboard.battery_low_warning = False
        return pt.common.Status.SUCCESS


class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """

    def __init__(self, topic_name: str = "/scan",
                 name: str = 'Scan2BB',
                 safe_range: float = 0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan': 'ranges'},
                         # to decide when data should be cleared/reset.
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))

        self.blackboard.register_key(
            key='collison_warning', access=pt.common.Access.WRITE)
        self.blackboard.register_key(
            key='point_at_min_dist', access=pt.common.Access.WRITE)
        self.blackboard.collison_warning = False
        self.blackboard.point_at_min_dist = 0.0

        self.safe_range = safe_range

    def update(self):
        status = super(LaserScan2bb, self).update()

        if status != pt.common.Status.RUNNING:
            segment_size = 20
            laser_distances = np.array(self.blackboard.laser_scan)
            full_laser_distance = np.full((580), 50, dtype=float)
            full_laser_distance[:len(laser_distances)] = laser_distances
            laser_distances = full_laser_distance
            laser_distances = np.where(
                laser_distances != 0, laser_distances, 20.0)
            laser_distances = laser_distances.reshape(segment_size, -1)

            distance_segment_mean = np.mean(laser_distances, axis=1)
            min_distance = np.min(distance_segment_mean)
            self.blackboard.point_at_min_dist = min_distance
            if min_distance < self.safe_range:
                self.blackboard.collison_warning = True
                self.logger.info("[LASER SCAN] update: possible collison detected at [%0.3f meters]" %
                                 min_distance)
            else:
                self.blackboard.collison_warning = False
        return pt.common.Status.SUCCESS


class checkBlackboardVariable(pt.behaviour.Behaviour):
    def __init__(self, name="Check Blackboard variable", variable_name="dummy", expected=True):
        super(checkBlackboardVariable, self).__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key=variable_name, access=pt.common.Access.READ)
        self.variable_name = variable_name
        self.expected = expected

    def update(self):
        if self.blackboard.get(self.variable_name) == self.expected:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


def create_root() -> pt.behaviour.Behaviour:
    """Structures a behaviour tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))

    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    idle = pt.behaviours.Running(name="Idle")
    laser_scan2bb = LaserScan2bb()
    battery_status2bb = BatteryStatus2bb()

    collison_emergency = pt.composites.Selector(
        name="CollisonEmergency", memory=False)

    is_no_collison = checkBlackboardVariable(
        name="NoCollison?",
        variable_name="collison_warning",
        expected=False
    )
    failure_is_success_collision = pt.decorators.Inverter(
        name="Inverter",
        child=collison_emergency
    )

    battery_empty = pt.composites.Selector(name="Battery_Empty", memory=False)
    is_battery_ok = checkBlackboardVariable(
        name="BatteryOk?",
        variable_name="battery_low_warning",
        expected=False
    )

    failure_is_success_battery = pt.decorators.Inverter(
        name="Inverter",
        child=battery_empty
    )

    stop_motion = StopMotion()
    rotate = Rotate()

    root.add_children([topics2BB, priorities])

    topics2BB.add_children([battery_status2bb, laser_scan2bb])
    priorities.add_children(
        [failure_is_success_collision, failure_is_success_battery, idle])
    collison_emergency.add_children(
        [is_no_collison, stop_motion])
    battery_empty.add_children(
        [is_battery_ok, rotate])

    return root


def main():
    """Initialises and executes the behaviour tree
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(
            console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    # frequency of ticks
    tree.tick_tock(period_ms=100)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
