import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, file_path)
    
    try:
        with open(full_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f'Failed to load {full_path}: {str(e)}')


def generate_launch_description():
    # Get package share directories
    barrel_slam_share = get_package_share_directory('barrel_slam')
    cart_launch_share = get_package_share_directory('cart_launch')
    simple_controller_share = get_package_share_directory('simple_controller')
    controller_config = load_yaml('simple_controller', 'launch/controller.yaml')

    # SLAM node
    slam_node = Node(
        package='barrel_slam',
        executable='slam_node',
        name='ekf_slam',
        output='screen',
        remappings=[
            ('/scan', '/base_scan'),
            # ('/odom', '/robot/odom')
        ]
    )

    # Cart stage launch
    cart_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(cart_launch_share, 'launch', 'cart_stage.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(cart_launch_share, 'stage_worlds', 'kalman_map.world')
        }.items()
    )

    # Controller node
    controller_node = Node(
        package='simple_controller',
        executable='controller_node',
        name='controller',
        output='log',
        parameters=[
            {'proportional': controller_config['proportional']},
            {'differential': controller_config['differential']},
            {'integral': controller_config['integral']},
            {'max_antiwindup_error': controller_config['max_antiwindup_error']},
            {'radius': controller_config['radius']},
            {'cy': controller_config['cy']},
            {'traj_length': controller_config['traj_length']},
            {'timer_period': controller_config['timer_period']}
        ]
    )

    # Velocity publisher 
    vel_node = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/velocity', 'std_msgs/Float32', 'data: 2.0']
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(barrel_slam_share, 'launch', 'slam.rviz')]
    )

    return LaunchDescription([
        slam_node,
        cart_stage,
        controller_node,
        vel_node,
        rviz_node
    ])