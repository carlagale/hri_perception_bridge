"""HRI con el TIAGo (robot real o simulacion).

Las imagenes llegan del robot (Zenoh + topic_tools relay) como /camera/image_raw
y /camera/camera_info. El arbol TF lo publica el robot en /tf y /tf_static, asi
que NO se lanza camara local ni TF estatica.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    face_share = get_package_share_directory('hri_face_detect')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Arrancar RViz2')

    # 1. Detector de caras (frame optico de la camara de la cabeza del TIAGo)
    face_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(face_share, 'launch', 'face_detect.launch.py')
        ),
        launch_arguments={
            'rgb_camera': 'camera',
            'filtering_frame': 'head_front_camera_rgb_optical_frame'
        }.items()
    )

    # 2. Detector de cuerpos (MediaPipe)
    body_node = LifecycleNode(
        package='hri_body_detect',
        executable='hri_body_detect',
        name='hri_body_detect',
        namespace='',
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    body_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == body_node,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    body_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=body_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda n: n == body_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        ))],
        handle_once=True
    ))

    # 3. Reconocedor de emociones
    emotion_node = LifecycleNode(
        package='hri_emotion_recognizer',
        executable='hri_emotion_recognizer',
        name='hri_emotion_recognizer',
        namespace='',
        parameters=[{
            'emotion_model': 'emotion-ferplus-8.onnx',
            'model': 'emotion-ferplus-8.onnx'
        }],
        output='screen'
    )

    emotion_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == emotion_node,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    emotion_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=emotion_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda n: n == emotion_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        ))],
        handle_once=True
    ))

    # 4. Gestor de personas HRI
    person_manager_node = Node(
        package='hri_person_manager',
        executable='hri_person_manager',
        name='hri_person_manager',
        output='screen'
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        face_launch,
        body_node,
        body_configure,
        body_activate,
        emotion_node,
        emotion_configure,
        emotion_activate,
        person_manager_node,
        rviz_node,
    ])
