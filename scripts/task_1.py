#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from std_msgs.msg import String
import time

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')
        
        # Initialize navigation
        self.nav = BasicNavigator()
        
        # Create poses using the same coordinates from way_point.py
        self.poses = {
            #'origin': self.create_pose_stamped(0.0, 0.0, 0.0), 
            'home': self.create_pose_stamped(-0.4889, -1.9888, 1.5972),      # Dock position
            'kitchen': self.create_pose_stamped(-1.4575, 2.0320, -2.2591),  # Kitchen position
            'table1': self.create_pose_stamped(0.3952, 0.3808, 1.3949), # Table 1
            'table2': self.create_pose_stamped(0.1388, -1.6643, 0.0526), # Table 2
            'table3': self.create_pose_stamped(2.4747, 0.4734, 1.6459),  # Table 3
        }
        
        # Set initial pose as home position
        self.nav.setInitialPose(self.poses['home'])
        
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 is now active")
        
        # Create publisher for status updates
        self.status_publisher = self.create_publisher(
            String,
            'butler_status',
            10)
        
        # Subscribe to order topic
        self.order_subscription = self.create_subscription(
            String,
            'order',
            self.order_callback,
            10)
            
        self.get_logger().info("Butler Robot initialized and ready to receive orders")
        self.publish_status("Robot ready at home position")
        
    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose
    
    def publish_status(self, status_msg):
        msg = String()
        msg.data = status_msg
        self.status_publisher.publish(msg)
        self.get_logger().info(status_msg)
        
    def navigate_to(self, location):
        self.get_logger().info(f"Navigating to {location}")
        self.publish_status(f"Moving to {location}")
        
        self.nav.goToPose(self.poses[location])
        
        # Wait for task completion
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            time.sleep(0.1)
        
        # Get result of navigation
        result = self.nav.getResult()
        self.get_logger().info(f"Navigation to {location} completed with result: {result}")
        return result
    
    def order_callback(self, msg):
        order_data = msg.data.strip().lower()
        
        self.get_logger().info(f"Received order message: '{order_data}'")
        
        if order_data not in ['table1', 'table2', 'table3']:
            self.get_logger().error(f"Invalid table: {order_data}")
            self.publish_status(f"Error: Invalid table {order_data}")
            return
        
        self.get_logger().info(f"Order received for {order_data}")
        self.publish_status(f"Order received for {order_data}")
        
        self.deliver_order(order_data)
    
    def deliver_order(self, table):
        self.publish_status(f"Starting delivery to {table}")
        
        kitchen_result = self.navigate_to('kitchen')
        if "SUCCEEDED" not in str(kitchen_result).upper():
            self.get_logger().error("Failed to reach kitchen. Returning home.")
            self.navigate_to('home')
            self.publish_status("Delivery failed. Returned to home.")
            return
        
        self.publish_status("Food collected from kitchen")
        
        table_result = self.navigate_to(table)
        if "SUCCEEDED" not in str(table_result).upper():
            self.get_logger().error(f"Failed to reach {table}. Returning home.")
            self.navigate_to('home')
            self.publish_status("Delivery failed. Returned to home.")
            return
        
        self.publish_status(f"Food delivered to {table}")
        
        home_result = self.navigate_to('home')
        if "SUCCEEDED" in str(home_result).upper():
            self.publish_status("Delivery completed successfully. Robot at home position.")
        else:
            self.get_logger().error("Failed to return home.")
            self.publish_status("Failed to return to home position.")

def main():
    rclpy.init()
    
    butler_node = ButlerRobot()
    
    try:
        rclpy.spin(butler_node)
    except KeyboardInterrupt:
        butler_node.get_logger().info("Butler Robot service terminated")
    finally:
        butler_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()