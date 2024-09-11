# !~/ros2_ws/src/my_robot_voice/venv/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
from hri_audio_msgs.action import SpeechToText  # Import your custom action
import speech_recognition as sr
import threading
import time

class VoiceListenerActionServer(Node):

    def __init__(self):
        super().__init__('voice_listener_action_server')
        self._action_server = ActionServer(
            self,
            SpeechToText,
            'speech_to_text',
             execute_callback=self.execute_callback,
        )
        self.recognizer = sr.Recognizer()

        # Set the pause threshold to a lower value for quicker response
        self.recognizer.pause_threshold = 0.8  # Adjust this value as needed

        self.get_logger().info('VoiceListener action server has been started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Initialize the recognized_text attribute
        self.recognized_text = ''

        # Start the listening thread
        listening_thread = threading.Thread(target=self.listen_for_speech, args=(goal_handle,))
        listening_thread.start()


        # Start the feedback thread
        feedback_thread = threading.Thread(target=self.send_feedback, args=(goal_handle,))
        feedback_thread.start()

        listening_thread.join()
        feedback_thread.join()

        result = SpeechToText.Result()
        result.header = Header()
        result.recognized_text = self.recognized_text
        result.header.stamp = self.get_clock().now().to_msg()  # Optionally set a timestamp
        
        # Mark the goal as successful
        goal_handle.succeed()
        return result 

    def listen_for_speech(self, goal_handle):
        # Initialize the recognized_text attribute
        
        with sr.Microphone() as source:
            # Adjust for ambient noise and calibrate the microphone
            self.recognizer.adjust_for_ambient_noise(source)

            self.get_logger().info("Listening for directions...")

            try:
                # Listen for speech with a maximum timeout of 20 seconds
                audio = self.recognizer.listen(source, timeout=20)

                # Recognize the speech
                self.recognized_text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'You said: {self.recognized_text}')

                if not goal_handle.is_active:
                    self.get_logger().info('Goal is no longer active.')
                    return  # Return if the goal is not active
                
               

            except sr.UnknownValueError:
                self.get_logger().warning("Google Speech Recognition could not understand the audio")
                goal_handle.abort()  # Abort on error

            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results from Google Speech Recognition service; {e}")
                goal_handle.abort()  # Abort on error

            except sr.WaitTimeoutError:
                self.get_logger().info("No speech detected within the timeout period.")
                goal_handle.abort()  # Abort after timeout

    def send_feedback(self, goal_handle):
        # Send feedback periodically while the recognition is happening (every 1 second)
        start_time = time.time()

        while (time.time() - start_time) < 20:  # Keep sending feedback for up to 20 seconds
            if goal_handle.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                break  # Stop sending feedback if the goal has succeeded or aborted

            feedback_msg = SpeechToText.Feedback()
            feedback_msg.header = Header()  # You can populate the header with a timestamp if needed
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info("Feedback sent while listening...")
            time.sleep(1)  # Send feedback every 1 second

def main(args=None):
    rclpy.init(args=args)
    voice_listener_action_server = VoiceListenerActionServer()
    rclpy.spin(voice_listener_action_server)

    voice_listener_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
