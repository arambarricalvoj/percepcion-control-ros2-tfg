import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bilbotiks_controller')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='bilbotiks_controller',
            executable='motorrak_roboclaw',
            name='motorrak_roboclaw',
            output='screen',
            parameters=[params_file]  
        ),
        Node(
            package='bilbotiks_controller',
            executable='servoak',
            name='servoak',
            output='screen',
            parameters=[params_file] 
        ),
        Node(
            package='bilbotiks_controller',
            executable='imu',
            name='imu_argitaratzailea',
            output='screen',
        ),
        Node(
            package='bilbotiks_controller',
            executable='pertzepzio_proba_wrapper',
            name='pertzepzio_proba_wrapper',
            output='screen',
            parameters=[params_file] 
        ),
        Node(
            package='bilbotiks_controller',
            executable='lidar',
            name='lidar',
            output='screen',
            parameters=[params_file] 
        )
    ])