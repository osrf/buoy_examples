import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pbcontrol'),
        'config',
        'pb_gain_scheduler.yaml'
        )

    node=Node(
        package = 'pbcontrol',
        name = 'pb_gain_scheduler',
        executable = 'gain_scheduler_node.py',
        parameters = [config]
    )

    ld.add_action(node)

    return ld
