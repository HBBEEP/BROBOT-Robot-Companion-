#!/usr/bin/python3
import rclpy
import speech_recognition as sr
from rclpy.node import Node
from std_msgs.msg import String

class brobot_voice_recognition(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.publisher_ = self.create_publisher(String, '/voice_message', 10)
        timer_period = 0.01  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        voice_input = self.get_voice_input()
        if voice_input:
            msg = String()
            msg.data = voice_input
            self.publisher_.publish(msg)

    def get_voice_input(self):
        recognizer = sr.Recognizer()

        with sr.Microphone() as source:
            print("Say something:")
            try:
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source, timeout=99)
                print("Processing...")
                text = recognizer.recognize_google(audio)
                return text.lower()  # Convert the recognized text to lowercase for case-insensitive comparison

            except sr.UnknownValueError:
                print("Sorry, I could not understand what you said.")
            except sr.RequestError as e:
                print(f"Could not request results from Google Web Speech API; {e}")

def main(args=None):
    rclpy.init(args=args)
    node = brobot_voice_recognition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
