#!/usr/bin/python3
import cv2, rclpy, os
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge
import numpy as np
import face_recognition

class brobot_face_detection(Node):
    def __init__(self):
        super().__init__('face_node')
        self.predict_face = None
        self.user_camera_detect = self.create_publisher(Bool, "/camera_detect", 10)
        self.image_sub = self.create_subscription(Image, '/video_frames', self.image_callback,10)
        self.create_timer(0.5, self.timer_callback)
        self.br = CvBridge()
        self.user_name = None
        self.user_get_name = self.create_publisher(String, "/username", 10)

        # Get encoded features for all saved images
        self.saved_pictures = "/home/hbbeep/brobot_ws/src/brobot_controller/user_picture"
        self.all_people_faces = {}

        # Initialize arrays of known face encodings and their names
        self.known_face_encodings = []
        self.known_face_names = []

        # Initialize some variables
        self.face_locations = []
        self.face_encodings = []
        self.face_names = []
        self.process_this_frame = True
        self.face_image = {}
        self.face_encoding = {}
        self.process_frame_counter = 0
        self.process_every_n_frames = 10  # Adjust this value based on your needs

        self.load_known_faces()

    def load_known_faces(self):
        
        for filename in os.listdir(self.saved_pictures):
            if (filename != "dummy_face.jpg"):
                self.person_face, extension = os.path.splitext(filename)
                self.known_face_names.append(self.person_face)
                self.face_image[self.person_face] = face_recognition.load_image_file(f"/home/hbbeep/brobot_ws/src/brobot_controller/user_picture/{self.person_face}.jpg")
                self.face_encoding[self.person_face] = face_recognition.face_encodings(self.face_image[self.person_face])[0]
                self.known_face_encodings.append(self.face_encoding[self.person_face])

    def timer_callback(self):  
        message = Bool()
        message_name = String()
        message_name.data = str(self.user_name)
        if (self.predict_face):
            message.data = True
            self.user_get_name.publish(message_name)
            self.user_name = None   
        else:
            message.data = False
        self.user_camera_detect.publish(message)

    def image_callback(self, msg:Image):
        """
        Callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        self.bb_face_detection(current_frame)


        cv2.imshow("face detection", current_frame)
        cv2.waitKey(1)

    def bb_face_detection(self, frame) -> None:
        # print(np.array(self.known_face_names).shape)
        # Only process every other frame of video to save time
        # if self.process_this_frame:
        if self.process_frame_counter % self.process_every_n_frames == 0:
            # Resize frame of video to 1/4 size for faster face recognition processing
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = np.ascontiguousarray(small_frame[:, :, ::-1])        


            # Find all the faces and face encodings in the current frame of video
            self.face_locations = face_recognition.face_locations(rgb_small_frame)
            self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)

            self.face_names = []
            for self.face_encoding in self.face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings, self.face_encoding)
                name = "Unknown"
                self.predict_face = False  
                # # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = self.known_face_names[first_match_index]
                    self.predict_face = True
                self.face_names.append(name)
                self.user_name = self.face_names

        # self.process_this_frame = not self.process_this_frame
        self.process_frame_counter += 1

        # Display the results
        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = brobot_face_detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
