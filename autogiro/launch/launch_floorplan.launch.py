import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():

    pkg_autogiro = get_package_share_directory('autogiro')
    # Build path to the specific world file
    world_file = os.path.join(pkg_autogiro, 'worlds', 'acopian_world.world')
    
    package_name='autogiro'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': f'-r {world_file}'}.items()
             )

    # Run the spawner node from the ros_gz_sim package
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'autogiro', 
                                   '-x', '13.05',
                                   '-y', '10.71',
                                   '-z', '0.5' 
                                   ],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    # MODIFIED: Changed /scan to /points
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Gazebo's gpu_lidar with <topic>points</topic> publishes the LaserScan
        # on /points and the PointCloud2 on /points/points — we want the cloud.
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    crop_self_node = Node(
        package='autogiro',
        executable='crop_self_hits.py',
        name='crop_self_hits',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ADDED: PointCloud to LaserScan converter
    pctols_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/points_filtered'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'laser_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.5,  
            'max_height': 1.0,  
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': False,
            'inf_epsilon': 1.0,
        }]
    )

    # Delay the spawners until spawn_entity has finished to ensure Gazebo is ready
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    # Launch them all
    return LaunchDescription([
        SetEnvironmentVariable('LC_ALL', 'en_US.UTF-8'),
        SetEnvironmentVariable('LANG', 'en_US.UTF-8'),
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner, 
        clock_bridge,
        crop_self_node,
        pctols_node # ADDED: Make sure to return the new node
    ])