from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    DeclareLaunchArgument('type', description="Current type: const, osc, or mod", choices=['const', 'osc', 'mod'])
    # DeclareLaunchArgument('vel', default_value = None),       # Req, for const
    # DeclareLaunchArgument('time', default_value = None),      # Req, for const
    # DeclareLaunchArgument('high', default_value = None),      # Req, for osc and mod
    # DeclareLaunchArgument('low', default_value = None),       # 0,   for osc and mod
    # DeclareLaunchArgument('ang', default_value = None),       # Req, for all
    # DeclareLaunchArgument('period', default_value = None),    # 6.0, for osc and mod
    # DeclareLaunchArgument('updraft', default_value = None),   # 0,   for all

    return LaunchDescription([
        OpaqueFunction(function=get_params)
    ])


def get_params(context):
    print("Getting relevant params...")

    type = LaunchConfiguration('type').perform(context)

    match type:
        case 'const':
            executable = 'constant_current'
        case 'osc':
            executable = 'oscillate_current'
        # case 'mod':
        #     executable = 'modulate_current'
        case _:
            print(f"Type must be: const, osc. Got: {type} - Rejecting!")
            return

    exec_path = os.path.join(
    get_package_prefix('enviro_disturbances'),
    'lib', 'enviro_disturbances', executable
    )

    cmd = [exec_path]

    match executable:
        case 'constant_current':
            cmd.extend(['-v', ask_for_input("Target velocity")])
            cmd.extend(['-a', ask_for_input("Angle from x-axis")])
            cmd.extend(['--updraft', ask_for_input("Updraft current velocity", 0.0)])
            cmd.extend(['-t', ask_for_input("Time to target velocity", 0.1, bottom_lim=0.0)])

        case 'oscillate_current':
            cmd.extend(['-v', ask_for_input("Top current velocity")])
            cmd.extend(['-l', ask_for_input("Bottom current velocity", 0)])
            cmd.extend(['-a', ask_for_input("Current angle")])
            cmd.extend(['-p', ask_for_input("Oscillation period", bottom_lim=0.2)])
            cmd.extend(['--updraft', ask_for_input("Updraft current velocity", 0.0)])

        case 'modulate_current':
            pass

    return [ExecuteProcess(cmd=cmd, output='screen')]


def ask_for_input(var_name, default_value = None, bottom_lim = None):

    default_val_str = "" if default_value is None else f"(default: {default_value})"

    while True:
        inp = input(f"Enter float value for {var_name} {default_val_str}: ")

        if default_value is not None:
            if not inp.strip():
                print("Setting default value")
                return str(default_value)

        try:
            v = float(inp)
        except ValueError:
            print("Invalid input!")
            continue

        if bottom_lim:
            if v < bottom_lim:
                print(f"{var_name} cannot be less than {bottom_lim}!")
                continue

        return str(v)