"""Launch de prueba rápida para verificar únicamente la ejecución de Poses del TIAGo.

No carga detectores de visión ni emociones. Solo levanta el nodo C++ PoseManager.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo Gestor de Poses en C++
    pose_tiago_node = Node(
        package='hri_perception_bridge',
        executable='pose_tiago_node',
        name='pose_manager',
        output='screen'
    )

    return LaunchDescription([
        pose_tiago_node
    ])