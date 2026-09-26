import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    bridge_share = get_package_share_directory('hri_perception_bridge')
    yunet_share = get_package_share_directory('hri_face_detect_yunet')
    
    camera_info_path = os.path.join(bridge_share, 'config', 'camera_info.yaml')
    model_path = 'emotion-ferplus-8.onnx'
    # 1. Driver de la cámara (gscam)
    camera_node = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_publisher',
        parameters=[{
            'gscam_config': 'v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1 ! videoconvert',
            'use_sensor_data_qos': True,
            'camera_name': 'camera',
            'frame_id': 'camera',
            'camera_info_url': f'file://{camera_info_path}',
            'use_sim_time': False
        }],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('camera/camera_info', '/camera/camera_info')
        ]
    )

    # 2. Detector de Caras (YuNet Launch Nativo)
    yunet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yunet_share, 'launch', 'hri_face_detect_yunet.launch.py')
        )
    )

    # 3. Lifecycle Node para Detector de Cuerpos (MediaPipe)
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

    # 4. Lifecycle Node para Reconocimiento de Emociones
    emotion_node = LifecycleNode(
        package='hri_emotion_recognizer',
        executable='hri_emotion_recognizer',
        name='hri_emotion_recognizer',
        namespace='',
        parameters=[{
            'emotion_model': model_path
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

    # 5. Gestor de Personas HRI
    person_manager_node = Node(
        package='hri_person_manager',
        executable='hri_person_manager',
        name='hri_person_manager',
        output='screen'
    )

    # 6. Visualizador RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.0', '0.0', '1.0',         # x, y, z (a 1m de altura)
            '-1.5708', '0.0', '-1.5708',  # yaw, pitch, roll (rotación óptica estándar)
            'base_link', 'camera'
        ]
    )

    return LaunchDescription([
        camera_node,
        yunet_launch,
        body_node,
        body_configure,
        body_activate,
        emotion_node,
        emotion_configure,
        emotion_activate,
        person_manager_node,
        rviz_node,
        tf_base_to_camera
    ])