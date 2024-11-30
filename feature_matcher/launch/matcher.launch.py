import os
import sys
import yaml

import launch
import launch_ros.actions
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
    controller_config = load_yaml('simple_controller', 'launch/controller.yaml')
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='simple_controller',
            executable='controller_node',
            name='controller',
            output='screen',
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
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/velocity', 'std_msgs/Float32', 'data: 2.0']
        ),
        launch_ros.actions.Node(
            package='feature_matcher',
            executable='feature_matcher',
            name='matcher',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(
                'feature_matcher'), 'launch/matcher.rviz')]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'cart_launch'), 'launch/cart_stage.launch.py')
            ),
            launch_arguments={
                'world': get_package_share_directory('cart_launch') + '/stage_worlds/feature_map_simple.world'
            }.items()
        )
    ])
    return ld


# if __name__ == '__main__':
#     generate_launch_description()
