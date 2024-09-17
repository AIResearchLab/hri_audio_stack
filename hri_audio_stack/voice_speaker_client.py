# !~/ros2_ws/src/my_robot_voice/venv/bin/python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hri_audio_msgs.action import TextToSpeech  # Your custom action
from std_msgs.msg import Header

class VoiceSpeakerClient(Node):

    def __init__(self):
        super().__init__('voice_speaker_client')

        # Initialize ActionClient with the custom TextToSpeech action
        self._action_client = ActionClient(self, TextToSpeech, 'text_to_speech')

    def send_goal(self, tts_text):
        self.get_logger().info(f'Sending goal: {tts_text}')

        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the action server to be available...')

        # Create the goal message
        goal_msg = TextToSpeech.Goal()
        goal_msg.header = Header()  # You can optionally set a timestamp here
        goal_msg.tts_text = tts_text

        # Send the goal to the action server
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the server.')
            return

        self.get_logger().info('Goal accepted by the server.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received from the server.')
        self.get_logger().info(f'Timestamp: {result.header.stamp.sec}.{result.header.stamp.nanosec}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received at timestamp: {str(feedback_msg.feedback)}')


def main(args=None):
    rclpy.init(args=args)

    client = VoiceSpeakerClient()

    # Get the text input from the user (can be replaced by any text source)
    text_to_speak = input("Enter the text you want to convert to speech: ")

    client.send_goal(text_to_speak)

    rclpy.spin(client)


if __name__ == '__main__':
    main()