import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from math import sin, cos


class GoToPose(Node):

    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Waypoints to explore
        self.waypoints = [
            (2.0, 1.0, 0.0),
            (-1.0, -4.5, 0.0),
            (2.5, -11.0, 0.0),
            (6.0, -10.0, 0.0)
        ]

        self.current_goal = 0

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.get_logger().info(f'Sending goal: {x}, {y}')
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Goal Reached')
        # Move to next waypoint
        self.current_goal += 1

        if self.current_goal < len(self.waypoints):
            x, y, yaw = self.waypoints[self.current_goal]
            self.send_goal(x, y, yaw)
        else:
            self.get_logger().info('Exploration finished')

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    go_to_pose = GoToPose()
    x, y, yaw = go_to_pose.waypoints[0]
    go_to_pose.send_goal(x, y, yaw)
    rclpy.spin(go_to_pose)

if __name__ == '__main__':
    main()