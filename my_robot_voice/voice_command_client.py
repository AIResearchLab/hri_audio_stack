#!/home/user/ros2_ws/src/my_robot_voice/venv/bin/python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_msgs.action import VoiceCommand
from rclpy.duration import Duration

class VoiceCommandClient(Node):
    def __init__(self):
        super().__init__('voice_command_client')
        self._action_client = ActionClient(self, VoiceCommand, 'voice_command')
        self.get_logger().info('VoiceCommand client has been started.')

    def send_goal(self):
        # Wait until the action server is available
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        self.get_logger().info('Action server available. Sending goal...')
        goal_msg = VoiceCommand.Goal()  # Create a new goal message
        goal_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
        
        # Send the goal to the action server
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        # Handle the response from the server (whether the goal was accepted or rejected)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted.')
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the server
        self.get_logger().info('Received feedback:')
        self.get_logger().info(str(feedback_msg.feedback))

    def result_callback(self, future):
        # Handle the result once it is available
        result = future.result().result
        self.get_logger().info('Received result:')
        self.get_logger().info(f'Recognized Text: {result.recognized_text}')

def main(args=None):
    rclpy.init(args=args)
    voice_command_client = VoiceCommandClient()
    voice_command_client.send_goal()
    
    try:
        rclpy.spin(voice_command_client)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS 2 client
        voice_command_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
