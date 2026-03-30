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
                    #launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
                    launch_arguments={'gz_args': f'-r {world_file}'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'autogiro', 
                                   '-x', '13.05',
                                   '-y', '10.71',
                                   '-z', '0.5' # Changed spawn position to be by stairs, also increased ground_plane dimensions to 150x150 
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
    
    clock_bridge = Node(
    	package='ros_gz_bridge',
   	 executable='parameter_bridge',
    	arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', 
    		   '/scan@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
    	output='screen'
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
        clock_bridge
    ])
