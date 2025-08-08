import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
    
def launch_setup(context, *args, **kwargs):
    package_name = LaunchConfiguration('package_name').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)

    package_directory = get_package_share_directory(package_name)
    config_file_path = os.path.join(package_directory, 'config', config_file)
    use_sim_time = use_sim_time_str.lower() in ['true', '1', 'yes']
    
    syncro_drive_node = Node(
        package='pointcloud_filter',
        executable='pointcloud_filter',
        output='screen',
        parameters=[config_file_path],
    )

    return [syncro_drive_node]

def generate_launch_description():
    declare_pkg_name = DeclareLaunchArgument(
        'package_name',
        default_value='pointcloud_filter',
        description='Name of the package containing the robot description'
    )
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='pointcloud_filter.yaml',
        description='Configuration file for the pointcloud filter node'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    return LaunchDescription([
        declare_pkg_name,
        declare_config_file,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])        