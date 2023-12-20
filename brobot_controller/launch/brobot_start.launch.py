from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    open_camera = Node(
        package='brobot_controller',
        executable='brobot_camera.py'
    )

    detect_face = Node(
        package='brobot_controller',
        executable='brobot_face_recognition.py'
    )

    detect_hand = Node(
        package='brobot_controller',
        executable='brobot_hand_gesture_recognition.py'
    )

    main_interact= Node(
        package='brobot_controller',
        executable='brobot_interact.py'
    )

    enable_speak = Node(
        package='brobot_controller',
        executable='brobot_speak.py'
    )

    enable_listen = Node(
        package='brobot_controller',
        executable='brobot_voice_recognition.py'
    )

    enable_current_timer_service = Node(
        package='brobot_controller',
        executable='brobot_current_timer.py'
    )


    launch_description = LaunchDescription()
    launch_description.add_action(open_camera)
    launch_description.add_action(detect_face)
    launch_description.add_action(detect_hand)
    launch_description.add_action(main_interact)
    launch_description.add_action(enable_speak)
    launch_description.add_action(enable_listen)
    launch_description.add_action(enable_current_timer_service)

    return launch_description
