import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_planning import a_star  # Import your path planning function

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal = (3, 2)
        self.path = a_star((0, 0), self.goal, self.load_map())
        self.current_target_index = 0
        self.pid_controller = PIDController()  # Ensure you have this implemented

    def load_map(self):
        # Load your map here
        pass

    def odom_callback(self, msg):
        # Get the current position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Calculate control commands
        linear_velocity, angular_velocity = self.pid_controller.calculate(current_x, current_y, self.path[self.current_target_index])
        
        # Publish the velocity commands
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.publisher.publish(msg)

        # Check if the current target is reached
        if self.is_goal_reached(current_x, current_y):
            self.current_target_index += 1
            if self.current_target_index >= len(self.path):
                self.stop_turtlebot()

    def is_goal_reached(self, x, y):
        # Check if the robot has reached the goal
        pass

    def stop_turtlebot(self):
        msg = Twist()  # Zero velocity
        self.publisher.publish(msg)
        self.get_logger().info('Goal Reached, Stopping TurtleBot')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

