import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import time

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Publisher for motor speeds
        self.left_motor_pub = self.create_publisher(Float32, 'RobileLeftMotorSpeed', 10)
        self.right_motor_pub = self.create_publisher(Float32, 'RobileRightMotorSpeed', 10)

        # Subscriber for sensor trigger
        self.sensor_sub = self.create_subscription(Bool, 'sensorTrigger', self.sensor_callback, 10)

        self.obstacle_detected = False
        self.state = 'DRIVE_FORWARD'
        self.backoff_start_time = None

        self.timer = self.create_timer(0.1, self.drive_robot)
        self.get_logger().info('ObstacleAvoidanceNode started.')

    def sensor_callback(self, msg):
        self.obstacle_detected = msg.data
        self.get_logger().info(f'Obstacle detected: {self.obstacle_detected}')

    def drive_robot(self):
        left_speed = Float32()
        right_speed = Float32()

        if self.state == 'DRIVE_FORWARD':
            if self.obstacle_detected:
                self.get_logger().info('Obstacle detected, switching to BACK_OFF state.')
                self.state = 'BACK_OFF'
                self.backoff_start_time = time.time()
            else:
                self.get_logger().info('Driving forward.')
                left_speed.data = 4.0
                right_speed.data = 4.0

        elif self.state == 'BACK_OFF':
            self.get_logger().info('Backing off.')
            left_speed.data = -10.0
            right_speed.data = -1.0
            if time.time() - self.backoff_start_time > 5: 
                self.get_logger().info('Back off complete, switching to DELAY_AFTER_BACK_OFF state.')
                self.state = 'DELAY_AFTER_BACK_OFF'
                self.delay_start_time = time.time()

        elif self.state == 'DELAY_AFTER_BACK_OFF':
            self.get_logger().info('Delaying after back off.')
            left_speed.data = 0.0
            right_speed.data = 0.0
            if time.time() - self.delay_start_time > 1: 
                self.get_logger().info('Delay complete, switching to DRIVE_FORWARD state.')
                self.state = 'DRIVE_FORWARD'

        self.left_motor_pub.publish(left_speed)
        self.right_motor_pub.publish(right_speed)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
