# !~/ros2_ws/src/my_robot_voice/venv/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_msgs.action import VoiceCommand  # Import your custom action
import speech_recognition as sr
import threading

class VoiceListenerActionServer(Node):

    def __init__(self):
        super().__init__('voice_listener_action_server')
        self._action_server = ActionServer(
            self,
            VoiceCommand,
            'voice_command',
            self.execute_callback,
            # feedback_msg_type=VoiceCommand.Feedback  # Feedback is included but not used
        )
        self.recognizer = sr.Recognizer()

        # Set the pause threshold to a lower value for quicker response
        self.recognizer.pause_threshold = 0.8  # Adjust this value as needed

        self.get_logger().info('VoiceListener action server has been started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Initialize the recognized_text attribute
        self.recognized_text = ''
        
        # Start the listening thread
        listening_thread = threading.Thread(target=self.listen_for_speech, args=(goal_handle,))
        listening_thread.start()

        # Wait for the thread to complete
        listening_thread.join()

        # Create and return the result
        result = VoiceCommand.Result()
        result.recognized_text = self.recognized_text
        return result

    def listen_for_speech(self, goal_handle):
        with sr.Microphone() as source:
            # Adjust for ambient noise and calibrate the microphone
            self.recognizer.adjust_for_ambient_noise(source)

            self.get_logger().info("Listening for directions...")
            audio = self.recognizer.listen(source)

            try:
                self.recognized_text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'You said: {self.recognized_text}')
                goal_handle.succeed()
            except sr.UnknownValueError:
                self.get_logger().error("Google Speech Recognition could not understand audio")
                # No explicit abort needed, just return
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results; {e}")
                # No explicit abort needed, just return

def main(args=None):
    rclpy.init(args=args)
    voice_listener_action_server = VoiceListenerActionServer()
    rclpy.spin(voice_listener_action_server)

    voice_listener_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
