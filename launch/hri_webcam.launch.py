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
    face_share = get_package_share_directory('hri_face_detect')

    camera_info_path = os.path.join(bridge_share, 'config', 'camera_info.yaml')

    # 1. Driver de la camara (gscam)
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

    # 2. Detector de Caras (Launch nativo de hri_face_detect con remapping a camara)
    face_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(face_share, 'launch', 'face_detect.launch.py')
        ),
        launch_arguments={
            'rgb_camera': 'camera',
            'filtering_frame': 'camera'
        }.items()
    )

    # 3. Detector de Cuerpos (MediaPipe)
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

    # 4. Reconocedor de Emociones
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

    # 5. Emparejador 2D Cara-Cuerpo (LifecycleNode)
    face_body_matcher_node = LifecycleNode(
        package='hri_face_body_matcher',
        executable='hri_face_body_matcher',
        name='hri_face_body_matcher',
        namespace='',
        output='screen'
    )

    face_body_matcher_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == face_body_matcher_node,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    face_body_matcher_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=face_body_matcher_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda n: n == face_body_matcher_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        ))],
        handle_once=True
    ))

    # 6. Gestor de Personas HRI (LifecycleNode)
    person_manager_node = LifecycleNode(
        package='hri_person_manager',
        executable='hri_person_manager',
        name='hri_person_manager',
        namespace='',
        output='screen'
    )

    person_manager_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == person_manager_node,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    person_manager_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=person_manager_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda n: n == person_manager_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        ))],
        handle_once=True
    ))

    # 7. Estimador de Atencion / Engagement (LifecycleNode)
    engagement_node = LifecycleNode(
        package='hri_engagement',
        executable='engagement',
        name='hri_engagement',
        namespace='',
        output='screen'
    )

    engagement_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == engagement_node,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    engagement_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=engagement_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda n: n == engagement_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        ))],
        handle_once=True
    ))

    # 8. Visualizador RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 9. Transformada TF estatica (base_link -> camera)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.0', '0.0', '1.0',
            '-1.5708', '0.0', '-1.5708',
            'base_link', 'camera'
        ]
    )

    return LaunchDescription([
        camera_node,
        face_launch,
        body_node,
        body_configure,
        body_activate,
        emotion_node,
        emotion_configure,
        emotion_activate,
        face_body_matcher_node,
        face_body_matcher_configure,
        face_body_matcher_activate,
        person_manager_node,
        person_manager_configure,
        person_manager_activate,
        engagement_node,
        engagement_configure,
        engagement_activate,
        rviz_node,
        tf_base_to_camera
    ])