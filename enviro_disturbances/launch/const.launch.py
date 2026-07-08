from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    executable = 'constant_current'

    exec_path = os.path.join(
    get_package_prefix('enviro_disturbances'),
    'lib', 'enviro_disturbances', executable
    )

    return LaunchDescription([
        DeclareLaunchArgument('vel', default_value='0.3'),
        DeclareLaunchArgument('t', default_value = '1.0'),
        DeclareLaunchArgument('ang', default_value='0'),
        DeclareLaunchArgument('up', default_value = '0'),

        ExecuteProcess(
            cmd = [
                exec_path, 
                '-v', LaunchConfiguration('vel'),
                '-a', LaunchConfiguration('ang'),
                '-t', LaunchConfiguration('t'),
                '--updraft', LaunchConfiguration('up')
            ],
            output='screen'
        )
    ])