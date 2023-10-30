import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    amcl_dir = get_package_share_directory('tb4_cpp_prac6')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    lifecycle_nodes = ['map_server', 'iar_amcl']

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation (Gazebo) clock if true' ),

        DeclareLaunchArgument('params_file', default_value=os.path.join(amcl_dir, 'config', 'localisation.yaml'),
                              description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument('map', default_value=os.path.join(amcl_dir, 'maps', 'lab_ICTE4001_Prac4.yaml'),
                              description='Full path to map yaml file to load'),

        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='tb4_cpp_prac6',
            executable='iar_amcl',
            name='iar_amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
        )
    ])
