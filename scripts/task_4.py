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
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time

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
            
        self.cancel_subscription = self.create_subscription(
            String,
            'cancel_order',
            self.cancel_callback,
            10)
            
        self.current_state = 'idle'  # Can be 'idle', 'to_kitchen', 'to_table', 'to_home'
        self.navigating = False
        self.cancellation_in_progress = False
        
        # Create a direct cancel client for nav2
        # Typically navigate_to_pose action would be at this topic
        self.cancel_client = self.create_client(
            CancelGoal, 
            '/navigate_to_pose/_action/cancel_goal'
        )
        
        # Wait for the cancel service to be available
        while not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for cancel service...')
            
        self.get_logger().info("Cancel service is available")
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
    
    def force_cancel_navigation(self):
        self.get_logger().info("Forcefully canceling navigation...")
        self.nav.cancelTask()
        req = CancelGoal.Request()

        future = self.cancel_client.call_async(req)
        
        self.get_logger().info("Cancel request sent to navigation server")
        self.publish_status("Navigation forcefully canceled")
        
        return True
        
    def navigate_to(self, location):
        """Navigate to a specific location and monitor for cancellation"""
        self.get_logger().info(f"Navigating to {location}")
        self.publish_status(f"Moving to {location}")
        
        if location == 'kitchen':
            self.current_state = 'to_kitchen'
        elif location in ['table1', 'table2', 'table3']:
            self.current_state = 'to_table'
        elif location == 'home':
            self.current_state = 'to_home'
            
        self.navigating = True
        self.cancellation_in_progress = False
            
        self.nav.goToPose(self.poses[location])
        
        while not self.nav.isTaskComplete():
            if self.cancellation_in_progress:
                self.get_logger().info(f"Cancellation detected during navigation to {location}")
                
                self.force_cancel_navigation()
                
                time.sleep(0.5)
                
                self.navigating = False
                return "CANCELED"
            time.sleep(0.2)

        result = self.nav.getResult()
        self.get_logger().info(f"Navigation to {location} completed with result: {result}")
        
        self.navigating = False
        return result
    
    def cancel_callback(self, msg):
        """Handle cancellation requests"""
        self.get_logger().info("Cancel order received!")
        
        if not self.navigating:
            self.get_logger().info("No active navigation to cancel")
            self.publish_status("No active navigation to cancel")
            return
            
        # Set the cancellation flag
        self.cancellation_in_progress = True
        current_state = self.current_state
        
        self.publish_status("Order cancellation in progress!")
        
        time.sleep(2.0)
        
        if self.navigating:
            self.get_logger().warn("Navigation did not cancel as expected, forcing stop")
            self.force_cancel_navigation()
            self.navigating = False
        
        if current_state == 'to_kitchen':
            self.publish_status("Cancellation while going to kitchen, returning to home")
            self.navigate_to('home')
        elif current_state == 'to_table':
            self.publish_status("Cancellation while going to table, returning via kitchen")
            kitchen_result = self.navigate_to('kitchen')
            if "SUCCEEDED" in str(kitchen_result).upper():
                self.navigate_to('home')
            else:
                self.navigate_to('home')
    
    def order_callback(self, msg):
        order_data = msg.data.strip().lower()
        
        self.get_logger().info(f"Received order message: '{order_data}'")
        
        if order_data not in ['table1', 'table2', 'table3']:
            self.get_logger().error(f"Invalid table: {order_data}")
            self.publish_status(f"Error: Invalid table {order_data}")
            return
        
        if self.navigating:
            self.get_logger().warning("Already processing an order, ignoring new order")
            self.publish_status("Already processing an order, ignoring new order")
            return
            
        self.get_logger().info(f"Order received for {order_data}")
        self.publish_status(f"Order received for {order_data}")
        
        delivery_thread = threading.Thread(target=self.deliver_order, args=(order_data,))
        delivery_thread.daemon = True
        delivery_thread.start()
    
    def deliver_order(self, table):
        self.publish_status(f"Starting delivery to {table}")
        
        kitchen_result = self.navigate_to('kitchen')
        if kitchen_result == "CANCELED":
            self.get_logger().info("Kitchen navigation was canceled")
            return
        
        if "SUCCEEDED" not in str(kitchen_result).upper():
            self.get_logger().error("Failed to reach kitchen. Returning home.")
            self.navigate_to('home')
            self.publish_status("Delivery failed. Returned to home.")
            return
        
        self.publish_status("Food collected from kitchen")
        
        table_result = self.navigate_to(table)
        
        if table_result == "CANCELED":
            self.get_logger().info("Table navigation was canceled")
            return
            
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