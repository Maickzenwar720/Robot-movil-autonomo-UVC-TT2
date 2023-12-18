from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
]

def generate_launch_description():

    pkg_diff_viz = get_package_share_directory('diff_pkg')

    rviz2_config = PathJoinSubstitution(
        [pkg_diff_viz, 'config', 'robot_navigation.rviz'])

    rviz = GroupAction([

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),

    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    return ld