from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    traj_gen = Node(
        package='crazyflie_trajectory',
        executable='traj_gen',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        traj_gen
    ])
