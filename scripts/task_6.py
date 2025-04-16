#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from std_msgs.msg import String
import time
import threading

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')
        
        # Initialize navigation
        self.nav = BasicNavigator()
        self.poses = {
            'home': self.create_pose_stamped(-0.4889, -1.9888, 1.5972),      # Dock position
            'kitchen': self.create_pose_stamped(-1.4575, 2.0320, -2.2591),  # Kitchen position
            'table1': self.create_pose_stamped(0.3952, 0.3808, 1.3949), # Table 1
            'table2': self.create_pose_stamped(0.1388, -1.6643, 0.0526), # Table 2
            'table3': self.create_pose_stamped(2.4747, 0.4734, 1.6459),  # Table 3
        }
        
        # Set initial pose as home position
        self.nav.setInitialPose(self.poses['home'])
        
        # Wait for Nav2 to be ready
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
            
        # Subscribe to confirmation topic
        self.confirmation_subscription = self.create_subscription(
            String,
            'confirmation',
            self.confirmation_callback,
            10)
        
        # Internal state variables
        self.awaiting_confirmation = False
        self.confirmation_received = False
        self.current_location = 'home'
        self.timeout_duration = 30.0  # 30 seconds timeout
        self.navigating = False
        self.delivery_in_progress = False
        
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
        
        self.navigating = True
        
        # Start navigation
        self.nav.goToPose(self.poses[location])
        
        # Wait for task completion
        while not self.nav.isTaskComplete():
            time.sleep(0.2)
        
        self.current_location = location
        
        result = self.nav.getResult()
        self.get_logger().info(f"Navigation to {location} completed with result: {result}")
        
        self.navigating = False
        return result
    
    def wait_for_confirmation(self, timeout=None):
        if timeout is None:
            timeout = self.timeout_duration
            
        self.awaiting_confirmation = True
        self.confirmation_received = False
        self.publish_status(f"Waiting for confirmation at {self.current_location}")
        
        start_time = time.time()
        while time.time() - start_time < timeout and not self.confirmation_received:
            time.sleep(0.1)
            
        result = self.confirmation_received
        self.awaiting_confirmation = False
        
        if result:
            self.get_logger().info(f"Confirmation received at {self.current_location}")
        else:
            self.get_logger().info(f"Confirmation timed out at {self.current_location}")
            
        return result
    
    def confirmation_callback(self, msg):
        self.get_logger().info(f"Confirmation message received: {msg.data}")
        
        if self.awaiting_confirmation:
            self.confirmation_received = True
            self.publish_status(f"Confirmation received at {self.current_location}")
            self.get_logger().info(f"Confirmation registered: {msg.data}")
        else:
            self.get_logger().info("Confirmation received but not currently awaiting confirmation")
    
    def order_callback(self, msg):
        order_data = msg.data.strip().lower()
        
        self.get_logger().info(f"Received order message: '{order_data}'")
        
        if self.delivery_in_progress:
            self.get_logger().warning("Already processing an order, ignoring new order")
            self.publish_status("Already processing an order, ignoring new order")
            return
        
        tables = []
        for table_num in order_data.split():
            table_id = f"table{table_num}"
            if table_id not in self.poses:
                self.get_logger().error(f"Invalid table: {table_id}")
                self.publish_status(f"Error: Invalid table {table_id}")
                continue
            tables.append(table_id)
        
        if not tables:
            self.get_logger().error("No valid tables specified in the order")
            self.publish_status("Error: No valid tables in the order")
            return
        
        self.get_logger().info(f"Order received for tables: {', '.join(tables)}")
        self.publish_status(f"Order received for tables: {', '.join(tables)}")
        
        delivery_thread = threading.Thread(target=self.deliver_with_confirmation, args=(tables,))
        delivery_thread.daemon = True
        self.delivery_in_progress = True
        delivery_thread.start()
    
    def deliver_with_confirmation(self, tables):
        
        try:
            table_list = ", ".join(tables)
            self.publish_status(f"Starting delivery to tables: {table_list}")
            
            kitchen_result = self.navigate_to('kitchen')
            if "SUCCEEDED" not in str(kitchen_result).upper():
                self.get_logger().error("Failed to reach kitchen. Returning home.")
                self.navigate_to('home')
                self.publish_status("Delivery failed. Returned to home.")
                return
            
            if not self.wait_for_confirmation():
                self.get_logger().error("No confirmation received at kitchen. Returning home.")
                self.navigate_to('home')
                self.publish_status("No confirmation at kitchen. Returned to home.")
                return
            
            self.publish_status(f"Food collected from kitchen for tables: {table_list}")
            
            delivered_tables = []
            
            for table in tables:
                table_result = self.navigate_to(table)
                if "SUCCEEDED" not in str(table_result).upper():
                    self.get_logger().error(f"Failed to reach {table}. Continuing with next table.")
                    continue
                
                if not self.wait_for_confirmation():
                    self.get_logger().warning(f"No confirmation received at {table}. Skipping.")
                    self.publish_status(f"No confirmation at {table}. Moving to next table.")
                    continue
                
                delivered_tables.append(table)
                self.publish_status(f"Food delivered to {table}")
            
            kitchen_result = self.navigate_to('kitchen')
            if "SUCCEEDED" not in str(kitchen_result).upper():
                self.get_logger().error("Failed to return to kitchen. Going home.")
                self.navigate_to('home')
                self.publish_status("Failed to return to kitchen. Returned home.")
                return
                
            self.publish_status("Delivery completed. Returned to kitchen.")
            
            home_result = self.navigate_to('home')
            if "SUCCEEDED" in str(home_result).upper():
                if delivered_tables:
                    tables_served = ", ".join(delivered_tables)
                    self.publish_status(f"Delivery completed for tables: {tables_served}. Robot at home position.")
                else:
                    self.publish_status("No tables were served. Robot returned to home position.")
            else:
                self.get_logger().error("Failed to return home.")
                self.publish_status("Failed to return to home position.")
        
        finally:
            self.delivery_in_progress = False

def main():
    rclpy.init()
    
    butler_node = ButlerRobot()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(butler_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        butler_node.get_logger().info("Butler Robot service terminated by keyboard interrupt")
    finally:
        executor.shutdown()
        butler_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()