#!/usr/bin/python3
import pyttsx3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Bool
from brobot_interfaces.action import SpeakAction
class brobot_speak(Node):
    def __init__(self):
        super().__init__('speak_action_server')
        self.engine = pyttsx3.init()
        self._action_server = ActionServer(self,
            SpeakAction,
            '/speak_action',
            self.execute_callback)


    def bb_speak(self, command) -> None:
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[3].id) # 0 (man) | 1 (girl)

        self.engine.setProperty('rate', 100)  # Adjust the speech rate
        self.engine.setProperty('volume', 1.0)  # Adjust the volume (1.0 is full volume)

        self.engine.say(command)
        self.engine.runAndWait()

    def execute_callback(self, speak_handle):      
        self.bb_speak(str(speak_handle.request.message))


        speak_handle.succeed()
        
       
        feedback_msg = SpeakAction.Feedback()
        feedback_msg.speakfeedback = True  
        speak_handle.publish_feedback(feedback_msg)

        result = SpeakAction.Result()
        result.speakresult = feedback_msg.speakfeedback
        return result

    

    
def main(args=None):
    rclpy.init(args=args)
    node = brobot_speak()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

