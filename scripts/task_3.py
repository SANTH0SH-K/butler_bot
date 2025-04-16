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

class ButlerRobotTask3(Node):
    def __init__(self):
        super().__init__('butler_robot_task3')
        
        self.nav = BasicNavigator()
        
        self.poses = {
            'home': self.create_pose_stamped(-0.4889, -1.9888, 1.5972),
            'kitchen': self.create_pose_stamped(-1.4575, 2.0320, -2.2591),
            'table1': self.create_pose_stamped(0.3952, 0.3808, 1.3949),
            'table2': self.create_pose_stamped(0.1388, -1.6643, 0.0526),
            'table3': self.create_pose_stamped(2.4747, 0.4734, 1.6459)
        }
        
        # Set initial pose
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
            
        # Initialize variables
        self.goals = []
        self.current_goal_index = 0
        self.waiting_for_confirmation = False
        self.confirmation_received = False
        self.timeout_duration = 10.0  
        self.current_location = 'home'
        
        self.publish_status("Robot ready at home position. Waiting for order...")
        self.get_logger().info("Butler Robot Task 3 initialized and ready to receive orders")
        
    def create_pose_stamped(self, position_x, position_y, rotation_z):
        """Create a PoseStamped message for the given coordinates"""
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def order_callback(self, msg):
        order_data = msg.data.strip().lower()
        
        self.get_logger().info(f"Received order message: '{order_data}'")
        
        if order_data not in ['table1', 'table2', 'table3']:
            self.get_logger().error(f"Invalid table: {order_data}")
            self.publish_status(f"Error: Invalid table {order_data}")
            return
        
        self.get_logger().info(f"Order received for {order_data}")
        self.publish_status(f"Order received for {order_data}")
        
        self.goals = ['kitchen', order_data, 'home']
        
        self.delivery_thread = threading.Thread(target=self.execute_delivery)
        self.delivery_thread.daemon = True
        self.delivery_thread.start()

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        self.get_logger().info(message)

    def navigate_to(self, location):
        self.get_logger().info(f"Navigating to {location}")
        self.publish_status(f"Moving to {location}")
        
        self.nav.goToPose(self.poses[location])
        
        while not self.nav.isTaskComplete():
            time.sleep(0.2)
        
        self.current_location = location
        
        result = self.nav.getResult()
        self.get_logger().info(f"Navigation to {location} completed with result: {result}")
        
        return result
    
    def wait_for_confirmation(self, timeout=None):
        if timeout is None:
            timeout = self.timeout_duration
            
        self.waiting_for_confirmation = True
        self.confirmation_received = False
        self.publish_status(f"Waiting for confirmation at {self.current_location}")
        
        start_time = time.time()
        while time.time() - start_time < timeout and not self.confirmation_received:
            time.sleep(0.1)
            
        result = self.confirmation_received
        self.waiting_for_confirmation = False
        
        if result:
            self.get_logger().info(f"Confirmation received at {self.current_location}")
        else:
            self.get_logger().info(f"Confirmation timed out at {self.current_location}")
            
        return result
        
    def execute_delivery(self):
        for i, goal in enumerate(self.goals):
            # Navigate to the goal
            nav_result = self.navigate_to(goal)
            
            # Check navigation result
            if "SUCCEEDED" not in str(nav_result).upper():
                self.get_logger().error(f"Failed to reach {goal}. Aborting mission.")
                if goal != 'home':
                    self.navigate_to('home')
                self.publish_status(f"Navigation to {goal} failed. Returned to home.")
                return
                
            if goal == 'home':
                self.get_logger().info('Returned to home. Mission complete.')
                self.publish_status('Mission complete. Robot at home position.')
                return
                
            # For kitchen and table, wait for confirmation
            self.get_logger().info(f'Waiting for confirmation at {goal}...')
            
            # Wait for confirmation or timeout
            if not self.wait_for_confirmation():
                self.get_logger().info(f'No confirmation received at {goal}.')
                
                # Handle timeout differently based on current location
                if goal == 'kitchen':
                    # If at kitchen, go directly home
                    self.publish_status('No confirmation received at kitchen. Returning to home.')
                    self.navigate_to('home')
                    self.publish_status('Mission aborted. Robot returned to home position.')
                else:
                    # If at table, go back to kitchen first, then home
                    self.publish_status('No confirmation received at table. Returning to kitchen then home.')
                    
                    # Go to kitchen first
                    kitchen_result = self.navigate_to('kitchen')
                    if "SUCCEEDED" in str(kitchen_result).upper():
                        self.publish_status('Back at kitchen. Now returning home.')
                    
                    # Then go home
                    self.navigate_to('home')
                    self.publish_status('Mission aborted. Robot returned to home position.')
                
                return

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ButlerRobotTask3()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        print('Navigation canceled by user!')
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()