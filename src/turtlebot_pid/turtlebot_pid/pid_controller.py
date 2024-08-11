import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # PID parameters
        self.goal_x = 5.0
        self.goal_y = 5.0
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

    def publish_velocity(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        error_x = self.goal_x - current_x
        error_y = self.goal_y - current_y

        distance_error = sqrt(error_x**2 + error_y**2)
        theta_error = atan2(error_y, error_x)

        # Calculate PID for linear velocity
        self.integral += distance_error
        derivative = distance_error - self.previous_error

        linear_velocity = self.kp * distance_error + self.ki * self.integral + self.kd * derivative
        angular_velocity = self.kp * theta_error  # Consider adding Ki and Kd if needed

        self.previous_error = distance_error

        # Publish the velocity command
        self.publish_velocity(linear_velocity, angular_velocity)

        # Stop when goal is reached
        if distance_error < 0.1:
            self.stop_turtlebot()

    def stop_turtlebot(self):
        msg = Twist()  # This message will have zero velocities
        self.publisher_.publish(msg)
        self.get_logger().info('Goal Reached, Stopping TurtleBot')

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

