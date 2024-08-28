#!~/ros2_ws/src/my_robot_voice/venv/bin/python3
import rclpy
from rclpy.node import Node
import speech_recognition as sr

class VoiceListener(Node):

    def __init__(self):
        super().__init__('voice_listener')
        self.recognizer = sr.Recognizer()

        # Set the pause threshold to a lower value for quicker response
        self.recognizer.pause_threshold = 0.8  # Adjust this value as needed
        
        self.get_logger().info('VoiceListener node has been started.')

    def get_speech_input(self):
        with sr.Microphone() as source:

            # Adjust for ambient noise and calibrate the microphone
            self.recognizer.adjust_for_ambient_noise(source)

            self.get_logger().info("Listening for directions...")
            audio = self.recognizer.listen(source)
            try:
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'You said: {text}')
                return text
            except sr.UnknownValueError:
                self.get_logger().error("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results; {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    voice_listener = VoiceListener()

    while rclpy.ok():
        voice_listener.get_speech_input()

    voice_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()