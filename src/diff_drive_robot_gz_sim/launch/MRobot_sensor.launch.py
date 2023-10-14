import xacro
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty',  description='Ignition World'),
    DeclareLaunchArgument('robot_name', default_value='ICTE4001_MRobot', description='Robot name'),
]

def generate_launch_description():

    pkg_diff_drive_robot_gz_sim = get_package_share_directory('diff_drive_robot_gz_sim')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    control_params_file = PathJoinSubstitution(
        [pkg_diff_drive_robot_gz_sim, 'config', 'mobile_base_control.yaml']
    )


    xacro_file = os.path.join(pkg_diff_drive_robot_gz_sim, 'urdf', 'ICTE4001_MRobot_sensor.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters = [params]
    )

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_diff_drive_robot_gz_sim, 'worlds')])

    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')]),
        launch_arguments=[('ign_args', [LaunchConfiguration('world'), '.sdf', ' -v 4'])]
    )

    rviz_config_file = os.path.join(pkg_diff_drive_robot_gz_sim, 'rviz', 'view_robot_sensor.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    spawn_robot = Node(
       package='ros_ign_gazebo', 
       executable='create',
       arguments = [
            '-name', LaunchConfiguration('robot_name'),
            '-z', '0.19', 
            '-topic', 'robot_description'
       ],
       output='screen'
    )

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', '/controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    ros_ign_bridge_launch_file = os.path.join(pkg_diff_drive_robot_gz_sim, 'launch', 'ros_gz_bridge.launch.py')
    ros_ign_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(ros_ign_bridge_launch_file), 
		launch_arguments=[
			('world', LaunchConfiguration('world')), 
			('robot_name', LaunchConfiguration('robot_name'))
		]
		
	)
    # LIDAR static transforms
    lidar_stf=Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='lidar_stf',
		output='screen',
		arguments=[
			'0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'lidar_link', [LaunchConfiguration('robot_name'), '/lidar_link/lidar_sensor']
		]
	)
    #IMU static transforms
    imu_stf=Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='imu_stf',
		output='screen',
		arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'imu_link', [LaunchConfiguration('robot_name'), '/imu_link/imu_sensor']]
	)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(rviz)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)
    ld.add_action(ros_ign_bridge)
    ld.add_action(lidar_stf)
    ld.add_action(imu_stf)
    return ld
