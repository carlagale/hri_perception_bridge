import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    bridge_share = get_package_share_directory('hri_perception_bridge')
    camera_info_path = os.path.join(bridge_share, 'config', 'camera_info.yaml')

    # 1. Nodo del driver de la cámara calibrado
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'camera_info_url': f'file://{camera_info_path}',
            'image_size': [640, 480],
            'output_encoding': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ]
    )

    # 2. Nodo Lifecycle de YuNet con remappings explícitos hacia tu cámara
    yunet_node = LifecycleNode(
        package='hri_face_detect_yunet',
        executable='hri_face_detect_yunet',
        name='hri_face_detect_yunet',
        namespace='',
        parameters=[{
            'confidence_threshold': 0.60,
            'image_scale': 0.5,
            'processing_rate': 30,
            'filtering_frame': 'camera'
        }],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    # 3. Automatización del Ciclo de Vida: Forzar TRANSITION_CONFIGURE al arrancar
    trigger_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(yunet_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # 4. Automatización del Ciclo de Vida: Forzar TRANSITION_ACTIVATE usando 'entities' para Jazzy
    trigger_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=yunet_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(yunet_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    # 5. Visor de imágenes integrado para validar que la cámara transmite
    viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='camera_viewer',
        remappings=[
            ('image', '/camera/image_raw')
        ]
    )

    return LaunchDescription([
        camera_node,
        yunet_node,
        trigger_configure,
        trigger_activate,
        viewer_node
    ])