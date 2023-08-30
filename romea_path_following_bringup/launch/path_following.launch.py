from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

from romea_mobile_base_description import get_mobile_base_description

import yaml


def load_control_parameters(filename):
    with open(filename) as file:
        return yaml.safe_load(file.read())


def launch_setup(context, *args, **kargs):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    control_file = LaunchConfiguration('control_config_file').perform(context)

    mobile_base_info = {'base': get_mobile_base_description(robot_type, robot_model)}
    control_params = load_control_parameters(control_file)

    return [
        LifecycleNode(
            package='romea_path_following',
            executable='path_following_node',
            name='path_following',
            exec_name='path_following',
            namespace=robot_type,
            parameters=[mobile_base_info, control_params],
            remappings=[
                # ('odometry', 'localisation/filtered_odom'),
            ],
            # prefix='terminator -x gdbserver localhost:1337',
            # prefix='terminator -x gdb --args',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('control_config_file'),
        OpaqueFunction(function=launch_setup)
    ])
