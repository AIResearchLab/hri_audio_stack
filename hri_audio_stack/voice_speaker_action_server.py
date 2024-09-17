# !~/ros2_ws/src/my_robot_voice/venv/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
from hri_audio_msgs.action import TextToSpeech  # Import your custom action
from gtts import gTTS
import threading
import time
import os


class VoiceSpeakerActionServer(Node):

    def __init__(self):
        super().__init__('voice_speaker_action_server')
        self._action_server = ActionServer(
            self,
            TextToSpeech,
            'text_to_speech',
             execute_callback=self.execute_callback,
        )

        self.get_logger().info('VoiceSpeaker action server has been started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Text-to-Speech goal...')

        # Get the text to convert to speech from the goal request
        text_to_speak = goal_handle.request.tts_text
        self.get_logger().info(f'Text to speak: {text_to_speak}')

        # Start the speaking and feedback threads
        speaking_thread = threading.Thread(target=self.speak_text, args=(goal_handle, text_to_speak))
        feedback_thread = threading.Thread(target=self.send_feedback, args=(goal_handle,))

        # Start both threads
        speaking_thread.start()
        feedback_thread.start()

        speaking_thread.join()
        feedback_thread.join()

        # Send the result after speech has been played
        result = TextToSpeech.Result()
        result.header = Header()
        result.header.stamp = self.get_clock().now().to_msg()  # Timestamp for the result

        return result

    def speak_text(self, goal_handle, text_to_speak):
        """Convert the received text to speech and play it."""
        try:
            # Generate speech using gTTS and save to a temporary file
            tts = gTTS(text=text_to_speak, lang='en')
            tts.save("temp_speech.mp3")

            # Play the saved audio file using a player (e.g., mpg321)
            os.system("mpg321 temp_speech.mp3")

            self.get_logger().info("Text-to-Speech conversion complete.")

            # Check if the goal is still active before marking it as successful
            if goal_handle.is_active:
                self.get_logger().info('Goal succeeded.')
                goal_handle.succeed()  # Mark the goal as successful

        except Exception as e:
            self.get_logger().error(f"Failed to generate speech: {e}")
            goal_handle.abort()

    def send_feedback(self, goal_handle):
        """Send feedback periodically while the text-to-speech conversion is in progress."""
        start_time = time.time()

        while (time.time() - start_time) < 10:  # Keep sending feedback for up to 10 seconds
            if goal_handle.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                break  # Stop sending feedback if the goal has succeeded or aborted

            feedback_msg = TextToSpeech.Feedback()
            feedback_msg.header = Header()  # Optionally populate the header with a timestamp
            feedback_msg.header.stamp = self.get_clock().now().to_msg()

            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info("Feedback sent during speech synthesis...")
            time.sleep(1)  # Send feedback every 1 second

def main(args=None):

    rclpy.init(args=args)
    voice_speaker_action_server = VoiceSpeakerActionServer()
    rclpy.spin(voice_speaker_action_server)

    voice_speaker_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
