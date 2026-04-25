import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class ProductLocator(Node):
    def __init__(self):
        super().__init__('product_locator')
        
        # Publisher to the standard Nav2 goal pose topic
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Product to aisle coordinate mapping
        # x is the horizontal center of the aisle, y is the specific aisle row
        self.product_database = {
            'beverages': {'x': 0.0, 'y': 1.75},
            'soda':      {'x': 0.0, 'y': 1.75},
            
            'snacks':    {'x': 0.0, 'y': -0.75},
            'chips':     {'x': 0.0, 'y': -0.75},
            
            'dairy':     {'x': 0.0, 'y': -3.25},
            'milk':      {'x': 0.0, 'y': -3.25},
            
            'household': {'x': 0.0, 'y': -5.50},
            'soap':      {'x': 0.0, 'y': -5.50},
            
            'checkout':  {'x': -6.5, 'y': 3.0}
        }

    def send_goal(self, product_name):
        # Wait a brief moment to ensure the publisher is fully registered with the ROS master
        import time
        time.sleep(1)

        product_name = product_name.lower()
        if product_name not in self.product_database:
            self.get_logger().error(f"Product '{product_name}' not found in the database.")
            self.get_logger().info("Available products: " + ", ".join(self.product_database.keys()))
            return False

        coords = self.product_database[product_name]
        
        # Create the PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.position.x = coords['x']
        goal_msg.pose.position.y = coords['y']
        goal_msg.pose.position.z = 0.0
        
        # Default orientation (facing positive X)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.get_logger().info(f"Locating '{product_name}'...")
        self.get_logger().info(f"Sending robot to Aisle Coordinates: ({coords['x']}, {coords['y']})")
        
        # Publish the goal
        self.goal_pub.publish(goal_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run supermarketbot locator <product_name>")
        print("Example: ros2 run supermarketbot locator milk")
        rclpy.shutdown()
        return

    product_name = sys.argv[1]
    
    node = ProductLocator()
    success = node.send_goal(product_name)
    
    if success:
        # Give it a moment to publish before spinning down
        rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
