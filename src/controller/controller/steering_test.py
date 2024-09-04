import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import math

class SteeringTest(Node):
    
    def __init__(self):
        super().__init__('steering_test')
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.drive_timer = self.create_timer(0.1, self.publish_drive)
        self.t = 0
        
    def publish_drive(self):
        ackermann_drive = AckermannDriveStamped()
        ackermann_drive.header.frame_id = 'base_link'
        ackermann_drive.header.stamp = self.get_clock().now().to_msg()
        ackermann_drive.drive.steering_angle = self.get_steering_angle()
        ackermann_drive.drive.steering_angle_velocity = 0.0
        ackermann_drive.drive.speed = 0.0
        ackermann_drive.drive.acceleration = 0.0
        ackermann_drive.drive.jerk = 0.0
        self.drive_pub.publish(ackermann_drive)
        
        self.get_logger().info(f'speed: {ackermann_drive.drive.steering_angle}')
        
    def get_steering_angle(self):
        d = math.sin(0.1*self.t)
        self.t += 0.1
        return d


def main(args=None):
    rclpy.init(args=args)
    speed_test = SteeringTest()
    rclpy.spin(speed_test)
    speed_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()