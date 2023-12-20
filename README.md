# Brobot
## _Your cherished new robot buddy_ ✨ 

Brobot is an interactive robot companion designed to enhance your entertainment experience. Ideally placed on a computer table, Brobot is programmed for user interaction through gestures and voice commands, providing an engaging and enjoyable experience.

## Project Overview

This project is a part of the FRA501 Robotics DevOps course. Our team includes:

1. Kullakant Kaewkallaya 
2. Natchanon Singha
3. Thamakorn Tongyod

## Features

- **Facial Recognition:** Identify users based on their faces.
- **Voice Interaction:** Communicate with Brobot using your own voice.
- **Gesture Control:** Command Brobot through hand gestures.


## RQT_GRPAH
![rosgraph](https://github.com/HBBEEP/brobot/assets/117287801/04ccb59d-1462-434d-8124-eabf6c8a9721)
### Nodes
#### 1. Camera Node

- **Description**: Captures images from the camera and publishes frames to the Hand and Face nodes through the `/video_frames` topic. 
- **Functionality**: Converts frames to ROS image messages before transmitting.

#### 2. Hand Node

- **Description**: Receives ROS image messages from the Camera node to classify hand gestures. 
- **Functionality**: Publishes results to the Interact node through the `/hand_detect` topic.

#### 3. Face Node

- **Description**: Receives ROS image messages from the Camera node to classify user faces.
- **Functionality**: Publishes face recognition results and status information to the Interact node through the `/user_name` and `/camera_detect` topics.

#### 4. Voice Node

- **Description**: Listens for voice input through the microphone.
- **Functionality**: Publishes received messages to the Interact node through the `/voice_message` topic.

#### 5. Interact Node

- **Description**: Responds to Brobot interactions and sends pose information to the Gazebo Simulation.
- **Functionality**: Handles inputs from Hand, Face, and Voice nodes. Integrates with the Speak Action Server and the Current Time Service.

### Services and Actions

#### 1. Speak Action (Speak_action_server)

- **Description**: Allows Brobot to generate speech.
- **Functionality**: Initiates the Speak action server to articulate responses.

#### 2. Current Time Service

- **Description**: Provides the current time as a string.
- **Functionality**: Enables the retrieval of real-time information about the current system time.

## Gazebo Simulation
###  Brobot model
The Brobot model can be viewed from the urdf file at "brobot_Gazebo/description". It is divided into 3 files:
- **manipulator.urdf.macro**
- **include.xacro**
- **gazebo.xacro**

In the file manipulator.urdf.xacro It determines the connections and joints of each part. 
In the 3D model, information can be extracted from "brobot_Gazebo/meshes/visual" this file is created from the blender.
It consist of

- **Body.stl**
- **arm.stl**
- **head.stl**
  
### Control scripts
To control the movement of Brobot in Gazebo, it is controlled with brobot_trajectory_publsiher node. It is a node that receives commands from /action_pub topic and passes conditions to select the trajectory to send to Brobot by pulling the trajectory from the BrobotPos.py file.
- **BrobotPos.py :** It is a file used to store various trajectories and functions
- **brobot_trajectory_publsiher.py :** It is a file for managing the movements of the Brobot according to the commands it receives.


## Prerequisites

In this code, we recommend using **ROS 2 Humble**, as it is the version we have utilized for development.

Before running this program, make sure to install the required libraries by using the following commands:

```bash
pip3 install scikit-learn==1.2.2
pip3 install opencv-python==4.8.1.78
pip3 install cv-bridge==3.2.1
pip3 install mediapipe==0.10.7
pip3 install face-recognition==1.3.0
```

## Important Reminder

### 1. Adding Your Face Picture
Please add your face to this folder (brobot_controller/user_picture) as (yourname.jpg) so the model can recognize you : )
### 2. Renaming path files
Please ensure to rename the path file to match your directory structure.
The relevant files for updating the directory are:
```
brobot_controller/brobot_face_recognition.py (line 23 & line 48)
brobot_controller/brobot_hand_gesture_recognition.py (line 16)
```
## Installation

1. Clone this repository to your desired directory:
```bash
git clone https://github.com/HBBEEP/brobot.git
```
2. Place all the packages in the src folder.
3. Follow these steps to build and set up the project in your workspace:
```bash
cd ~/[your_workspace]
colcon build
source install/setup.bash
```

## Implementation

### Running the Back-End:
To launch the back-end code, use the following command:
```bash
ros2 launch brobot_controller brobot_start.launch.py
```
### Running the Model and Simulation in Gazebo:
To run the model and initiate the simulation in Gazebo, execute the following command:
```bash
ros2 launch brobot_Gazebo Gazebo_robot.launch.py
```

These commands will set up and start the back-end and simulation components of Brobot.

## How to control Brobot by your voice and your face 
The brobot can simply control by both hand gesture and your own voice !
### Example :
#### Speak to your microphone 
| User Action (Say the word) | Brobot Response |
| ------ | ------ |
| "Hello" |  Brobot will change the pose in Gazebo Simulation and speak "Hello say hi from Brobot, I will take care of you" |
| "Brobot" , "Robot", "Bot"  |  Brobot will change the pose in Gazebo Simulation and speak "Brobot is here, here with you" |
| "Food" |  Brobot will change the pose in Gazebo Simulation and speak  "Brobot love food and food love brobot"|

#### Show your Hand Gesture 
Reference of the hand gesture commands (Brobot utilizes the Mediapipe framework for hand gesture recognition) 
| Hand gesture | meaning |
| ------ | ------ |
| 1 ![brobota1](https://github.com/HBBEEP/brobot/assets/117287801/22d1949f-9b2e-4a2a-962d-4d2f507beaec)| One finger Gesture |
| 2 ![brobota2](https://github.com/HBBEEP/brobot/assets/117287801/7f418b41-8503-436c-8c10-153ba50e93b7)| Two fingers Gesture |
| 3 ![brobota3](https://github.com/HBBEEP/brobot/assets/117287801/05fc458f-fec6-45f2-a1c9-5ae096f5d9c2)| Three fingers Gesture |
| 4 ![brobota4](https://github.com/HBBEEP/brobot/assets/117287801/1aeaf4f2-8e77-45f6-84ef-53ec27c44877)| Four fingers Gesture|
| 5 ![brobota5](https://github.com/HBBEEP/brobot/assets/117287801/c4170ed4-69db-4293-bde9-a65ca6614016)| Five fingers Gesture |
| 6 ![brobota6](https://github.com/HBBEEP/brobot/assets/117287801/a046cc79-f135-41c8-ac3a-ca43acd28bd8)| Good Gesture |
| 7 ![brobota7](https://github.com/HBBEEP/brobot/assets/117287801/a86af85c-1efb-4556-bb8f-2f24e1866f84)| Love Gesture |

There are seven commands of hand gestures for controlling the brobot (you can see the reference of the hand gesture commands above)
| User Action (Show hand gesture) | Brobot Response |
| ------ | ------ |
| 1. One finger Gesture | Brobot will change the pose in Gazebo Simulation and speak "Brobot Brobot Brobot Brobot"  |
| 2. Two fingers Gesture   | Brobot will change the pose in Gazebo Simulation and speak "Hey, This is me brobot, anything I can help you" |
| 3. Three finger Gesture | Brobot will change the pose in Gazebo Simulation and speak "Hello {your name} How are you" |
| 4. Four fingers Gesture   | Brobot will change the pose in Gazebo Simulation and speak the current time|
| 5. Five fingers Gesture  |Brobot will change the pose in Gazebo Simulation and speak ""Brobot high five"" |
| 6. Good Gesture  | Brobot will change the pose in Gazebo Simulation and speak "Robotics DevOps is Good, Brobot loves Robotics DevOps" |
| 7. Love Gesture | Brobot will change the pose in Gazebo Simulation and will speak ""Brobot love {your name} too"   |

## Demo
![brobot](https://github.com/HBBEEP/brobot/assets/117287801/92b26ba5-43fc-4fb7-bea2-1bb0f38d4566)

## Room for Improvement

1. Gesture Recognition Sensitivity

**Issue:** Brobot exhibits high sensitivity to changes in hand gestures,.

Potential Solution: Apply filtering techniques or algorithms to reduce the system's sensitivity to minor variations in hand gestures. This could enhance the accuracy and reliability of hand gesture recognition.

2. Intelligent Response Mechanism

**Issue:**  Brobot currently relies on a simplistic if-else logic for responding to user voice input. This approach limits the system's ability to handle diverse and complex interactions.

Potential Solution: Implement a more intelligent response mechanism. Consider incorporating natural language processing (NLP) or any other techniques to enable Brobot to understand and respond to a broader range of user queries.

