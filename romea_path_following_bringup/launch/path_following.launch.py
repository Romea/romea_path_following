from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', description="robot namespace", default_value=''),
        LifecycleNode(
            package='romea_path_following',
            executable='path_following_node',
            name='path_following',
            exec_name='path_following',
            namespace=LaunchConfiguration('robot_ns'),
            parameters=[{
                # 'autostart': True,
            }],
            remappings=[
                # ('odometry', 'localisation/filtered_odom'),
            ],
            # prefix='terminator -x gdbserver --no-startup-with-shell localhost:1337',
        )
    ])
