#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String 
from std_msgs.msg import Int8

from rclpy.action import ActionClient

from brobot_interfaces.action import SpeakAction

from brobot_interfaces.srv import CurrentTimeService

class brobot_interact(Node): 
    def __init__(self):
        super().__init__('interact_node')
        self.camera_detect = None
        self.hand_detect = None
        self.username = None 
        self.is_speaking = False

        self.create_subscription(Bool, "/camera_detect", self.camera_detect_callback, 10)
        self.create_subscription(Int8, "/hand_detect", self.hand_detect_callback,  10)
        self.create_subscription(String, "/username", self.get_user_name_callback,  10)
        self.create_subscription(String,  "/voice_message" , self.voice_recognition_callback,  10)

        self.brobotAction = self.create_publisher(Int8,"/action_pub",10)
        self.create_subscription(Bool, "/action_status", self.action_Status, 10)
        
        self._speak_action_client = ActionClient(self, SpeakAction, '/speak_action')

        self.current_timer_client = self.create_client(CurrentTimeService, '/current_time_service' )
        while not self.current_timer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Current Timer Service not available, waiting again...')

        self.current_timer_req = CurrentTimeService.Request()

    def voice_recognition_callback(self, msg):
        self.get_logger().info(f'Get Voice Data: {msg.data} ' ) 
        if "hello" in msg.data:
            talk_msg = "Hello say hi from Brobot, I will take care of you" 
            self.send_speak_action(str(talk_msg))
            self.action_control(1)
        elif "robot" or "brobot" or "bot" in msg.data:
            talk_msg = "brobot wake up brobot start"
            self.send_speak_action(str(talk_msg))
            self.action_control(1)
        elif "food" in msg.data:
            talk_msg = "brobot love food and food love brobot"
            self.send_speak_action(str(talk_msg))
            self.action_control(7)
        else:
            print("Specific word not detected. Listening again...")

    def get_user_name_callback(self, msg):
        self.username = msg.data
        # self.get_logger().info(f'Get Username Data: {self.username } ' ) 

    def camera_detect_callback(self, msg):
        self.camera_detect = msg.data
        # self.get_logger().info(f'Get Face Detect Data: {self.camera_detect } ' ) 


    def hand_detect_callback(self, msg): # interaction occurs here 
        self.hand_detect = msg.data
        if (self.is_speaking == False): 
            if (self.hand_detect == 1): # Speak brobot 
                self.is_speaking = True

                self.action_control(1)
                talk_msg = "brobot brobot brobot brobot"
                self.send_speak_action(str(talk_msg))

            elif (self.hand_detect == 2): # Speak like speaker
                self.is_speaking = True

                self.action_control(2)
                talk_msg = "Hey, This is me brobot, anything I can help you"
                self.send_speak_action(str(talk_msg))

            elif (self.hand_detect == 3): # Speak Name
                self.is_speaking = True

                self.action_control(3)
                talk_msg = f"Hello {self.username} How are you"
                self.send_speak_action(str(talk_msg))

            elif (self.hand_detect == 4): # Brobot Timer
                self.is_speaking = True

                self.action_control(4)
                self.call_current_timer_service()

            elif (self.hand_detect == 5 ):
                self.is_speaking = True

                self.action_control(5)
                talk_msg = "Brobot high five"
                self.send_speak_action(str(talk_msg))

            elif (self.hand_detect == 6):                     
                self.is_speaking = True

                self.action_control(6)
                talk_msg = "Robotics DevOps is Good, Brobot loves Robotics DevOps"
                self.send_speak_action(str(talk_msg))
        
            elif (self.hand_detect == 7):
                self.is_speaking = True

                self.action_control(7)
                talk_msg = f"Brobot love {self.username} too"
                self.send_speak_action(str(talk_msg))

        

    def action_control(self,hand_action):
        hand_action_publish = Int8()
        hand_action_publish.data = hand_action
        self.brobotAction.publish(hand_action_publish)
        
    def action_Status(self,msg):
        pass

    # --- Current Timer Service 

    def current_timer_service_callback(self, future):
        self.send_speak_action(f"This is brobot timer {str(future.result().message)}")
        
    def call_current_timer_service(self):
        self.future = self.current_timer_client.call_async(self.current_timer_req)
        self.future.add_done_callback(self.current_timer_service_callback)    

    # --- Speak Action 

    def send_speak_action(self, message):
        speak_msg = SpeakAction.Goal()
        speak_msg.message = message

        self._speak_action_client.wait_for_server()
        self._send_goal_future = self._speak_action_client.send_goal_async(speak_msg, feedback_callback=self.speak_feedback_callback)
        self._send_goal_future.add_done_callback(self.speak_response_callback)
    
    def speak_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.is_speaking = False


        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.speak_result_callback)

    def speak_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.speakresult))

    def speak_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.speakfeedback))

def main(args=None):
    rclpy.init(args=args)
    node = brobot_interact() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
