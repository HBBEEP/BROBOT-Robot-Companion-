#!/usr/bin/python3
import pickle
import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class brobot_hand_recognition(Node):
    def __init__(self):
        super().__init__('hand_node')
        self.model_dict = pickle.load(open('/home/hbbeep/brobot_ws/src/brobot_controller/model/model.p', 'rb'))
        self.model = self.model_dict['model']
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(static_image_mode=True, min_detection_confidence=0.3)
        self.labels_dict = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5, 5: 6, 6: 7}
        self.predicted_character = None
        self.msg = String()
        self.image_sub = self.create_subscription(Image, '/video_frames', self.image_callback,10)
        self.br = CvBridge()
        self.create_timer(2.0, self.timer_callback)
        self.user_get_name = self.create_publisher(String, "/user_name", 10)
        self.user_hand_detect = self.create_publisher(Int8, "/hand_detect", 10)


    def timer_callback(self):
        message = Int8()
        if (self.predicted_character != None):
            message.data = int(self.predicted_character)
            self.user_hand_detect.publish(message)

        self.predicted_character = None
        
   
    def image_callback(self, msg:Image):
        """
        Callback function.
        """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        self.bb_hand_gesture_predict(current_frame)
        # Display image
        cv2.imshow("hand_gesture", current_frame)
        
        cv2.waitKey(1)

    def bb_hand_gesture_predict(self, frame) -> None:
        data_aux = []
        x_ = []
        y_ = []
        H, W, _ = frame.shape

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.hands.process(frame_rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,  # image to draw
                    hand_landmarks,  # model output
                    self.mp_hands.HAND_CONNECTIONS,  # hand connections
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())
            
            for hand_landmarks in results.multi_hand_landmarks:
                for i in range(len(hand_landmarks.landmark)):
                    x = hand_landmarks.landmark[i].x
                    y = hand_landmarks.landmark[i].y

                    x_.append(x)
                    y_.append(y)

                for i in range(len(hand_landmarks.landmark)):
                    x = hand_landmarks.landmark[i].x
                    y = hand_landmarks.landmark[i].y
                    data_aux.append(x - min(x_))
                    data_aux.append(y - min(y_))

            x1 = int(min(x_) * W) - 10
            y1 = int(min(y_) * H) - 10

            x2 = int(max(x_) * W) - 10
            y2 = int(max(y_) * H) - 10
            if len(data_aux) != 84:
                prediction = self.model.predict([np.asarray(data_aux)])
                self.predicted_character = self.labels_dict[int(prediction[0])]
                
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)
            cv2.putText(frame, str(self.predicted_character), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 3, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = brobot_hand_recognition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
    
