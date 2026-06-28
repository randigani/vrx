from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    executable = 'oscillate_current'

    exec_path = os.path.join(
    get_package_prefix('enviro_disturbances'),
    'lib', 'enviro_disturbances', 'oscillate_current'
    )

    return LaunchDescription([
        DeclareLaunchArgument('high', description='Required!'),
        DeclareLaunchArgument('low', default_value = '0'),
        DeclareLaunchArgument('ang', description='Required!'),
        DeclareLaunchArgument('period', default_value = '6.0'),
        DeclareLaunchArgument('updraft', default_value = '0'),

        ExecuteProcess(
            cmd = [
                exec_path, 
                '-v', LaunchConfiguration('high'),
                '-l', LaunchConfiguration('low'),
                '-a', LaunchConfiguration('ang'),
                '-p', LaunchConfiguration('period'),
                '--updraft', LaunchConfiguration('updraft')
            ],
            output='screen'
        )
    ])