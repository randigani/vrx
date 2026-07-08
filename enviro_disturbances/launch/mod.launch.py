from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    executable = 'modulate_current'

    exec_path = os.path.join(
    get_package_prefix('enviro_disturbances'),
    'lib', 'enviro_disturbances', executable
    )

    return LaunchDescription([
        DeclareLaunchArgument('hi', default_value = '0.3'),
        DeclareLaunchArgument('lo', default_value = '0'),
        DeclareLaunchArgument('ang', default_value = '0'),
        DeclareLaunchArgument('pd', default_value = '6.0'),
        DeclareLaunchArgument('up', default_value = '0'),
        DeclareLaunchArgument('mod', default_value = '2.5'),


        ExecuteProcess(
            cmd = [
                exec_path, 
                '-v', LaunchConfiguration('hi'),
                '-l', LaunchConfiguration('lo'),
                '-a', LaunchConfiguration('ang'),
                '-p', LaunchConfiguration('pd'),
                '-m', LaunchConfiguration('mod'),
                '--updraft', LaunchConfiguration('up')
            ],
            output='screen'
        )
    ])