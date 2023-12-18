import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    default_params_file = os.path.join(get_package_share_directory("diff_pkg"),
                                       'config', 'mapper_params_online_async.yaml')
    # bringup_dir = get_package_share_directory('diff_pkg')

    # rviz2_config = PathJoinSubstitution(
    #     [bringup_dir, 'config', 'robot_mapping.rviz'])
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # rviz = GroupAction([
    #     Node(package='rviz2',
    #          executable='rviz2',
    #          name='rviz2',
    #          arguments=['-d', rviz2_config],
    #          parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #          output='screen'),
    # ])

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    # ld.add_action(rviz)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
