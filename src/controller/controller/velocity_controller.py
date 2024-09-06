import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math

class VelocityController(Node):
    
    def __init__(self):
        super().__init__('velocity_controller')
        self.velocity_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_velocity)
        self.goal_sub = self.create_subscription(PoseStamped, '/planner/goal', self.goal_callback, 10)
        self.sub_odom_ = self.create_subscription(Odometry, '/racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/racecar/drive', 10)

        self.goal_position = [0.0, 0.0]
        self.racecar_position = [0.0, 0.0]
        self.racecar_angle = 0.0
        
        # PD Controller parameters
        self.kp_lin = 1.0  # Proportional gain for linear velocity
        self.kd_lin = 0.1  # Derivative gain for linear velocity
        self.kp_rot = 1.5  # Proportional gain for angular velocity
        self.kd_rot = 0.2  # Derivative gain for angular velocity
        self.previous_trans_err = 0.0  # Previous translational error
        self.previous_rot_err = 0.0  # Previous rotational error
        self.previous_time = self.get_clock().now()  # Previous time
        self.v = 0.0  # Current linear velocity
        self.w = 0.0  # Current angular velocity
       
    def publish_velocity(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert to seconds

        # Compute errors
        forward = [math.cos(self.racecar_angle), math.sin(self.racecar_angle)]
        q = [self.goal_position[0] - self.racecar_position[0], self.goal_position[1] - self.racecar_position[1]]
        norm_q = math.sqrt(q[0]**2 + q[1]**2)
        
        if norm_q < 0.01:  # Dead band
            self.velocity_cmd_pub.publish(Twist())
            return
        
        projected_trans_err = q[0] * forward[0] + q[1] * forward[1]
        rot_err = math.atan2(q[1], q[0]) - self.racecar_angle
        rot_err = (rot_err + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # PD control
        trans_err_derivative = (projected_trans_err - self.previous_trans_err) / dt if dt > 0 else 0.0
        rot_err_derivative = (rot_err - self.previous_rot_err) / dt if dt > 0 else 0.0

        a = self.kp_lin * projected_trans_err + self.kd_lin * trans_err_derivative
        b = self.kp_rot * rot_err + self.kd_rot * rot_err_derivative

        # 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2,
        twist = Twist()
        twist.linear.x = max(min(a, 1.0), -0.5)  # Limit linear velocity
        twist.angular.z = max(min(b, 0.4), -0.4)  # Limit angular velocity
        # self.get_logger().info(f'{twist.angular.z}')
        # if twist.angular.z < abs(0.05):
        #     twist.angular.z = 0.0
        self.previous_trans_err = projected_trans_err
        self.previous_rot_err = rot_err
        self.previous_time = current_time

        # self.velocity_cmd_pub.publish(twist)
        
        # REAL CAR
        ackermann_drive = AckermannDriveStamped()
        ackermann_drive.header.frame_id = 'racecar/base_link'
        ackermann_drive.header.stamp = self.get_clock().now().to_msg()
        ackermann_drive.drive.steering_angle = twist.angular.z
        ackermann_drive.drive.steering_angle_velocity = 0.0
        ackermann_drive.drive.speed = 3.0
        ackermann_drive.drive.acceleration = 0.0
        ackermann_drive.drive.jerk = 0.0
        self.drive_pub.publish(ackermann_drive)
                
        
        
    def goal_callback(self, msg: PoseStamped):
        self.goal_position = [msg.pose.position.x, msg.pose.position.y]
        
    def odom_callback(self, msg: PoseWithCovarianceStamped):
        self.racecar_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        orientation_q = msg.pose.pose.orientation
        _, _, self.racecar_angle = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
