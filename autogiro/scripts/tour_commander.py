#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class TourCommander(Node):
    def __init__(self):
        super().__init__('tour_commander')
        
        # Publisher to trigger the tour slide module
        self.slide_trigger_pub = self.create_publisher(String, '/tour_interaction', 10)
        self.navigator = BasicNavigator()

    def create_pose(self, x, y, yaw_z=0.0, yaw_w=1.0):
        """Helper function to format coordinates into a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Using a simple quaternion for rotation (z, w)
        pose.pose.orientation.z = yaw_z
        pose.pose.orientation.w = yaw_w
        return pose

    def start_tour(self):
        # Wait for Nav2 to be fully online before sending commands
        self.navigator.waitUntilNav2Active()

        # Define your 4th-floor tour waypoints (You will need to adjust these X/Y values)
        origin = self.create_pose(0.0, 0.0)
        spot_1 = self.create_pose(5.0, 0.0)  # Example: 5 meters straight down the hall
        spot_2 = self.create_pose(5.0, 5.0, yaw_z=0.707, yaw_w=0.707) # Example: Turn left

        # Define the tour sequence and the slide to trigger at each stop
        tour_sequence = [
            {"pose": spot_1, "slide": "Welcome to the 4th Floor!"},
            {"pose": spot_2, "slide": "Here is the Robotics Lab."},
            {"pose": origin, "slide": "Tour complete. Returning to base."}
        ]

        self.get_logger().info("Starting the AUTOGIRO Tour!")

        for stop in tour_sequence:
            self.get_logger().info(f"Navigating to next stop...")
            
            # Send the goal to your Nav2 Behavior Tree
            self.navigator.goToPose(stop["pose"])

            # Wait for the robot to finish driving AND finish your BT's <Wait> node
            while not self.navigator.isTaskComplete():
                time.sleep(0.5)

            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Arrived! Triggering slide: {stop['slide']}")
                
                # Publish the message to your ROS 2 stack to trigger the slide
                msg = String()
                msg.data = stop["slide"]
                self.slide_trigger_pub.publish(msg)
                
                # Give the slide a moment to stay on screen before moving to the next waypoint
                time.sleep(2.0) 
            else:
                self.get_logger().error("Navigation failed! Aborting tour.")
                break

def main(args=None):
    rclpy.init(args=args)
    commander = TourCommander()
    commander.start_tour()
    rclpy.shutdown()

if __name__ == '__main__':
    main()