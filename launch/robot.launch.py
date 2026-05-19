import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():

    # Package name
    package_name='snowbot123'

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    # Path to default world 
    world_path = os.path.join(get_package_share_directory(package_name),'worlds', 'wow.sdf')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true',
        description='Use sim time if true')

    declare_nav2 = DeclareLaunchArgument(
        name='nav2', default_value='False',
        description='Launch Nav2 navigation stack if True')

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(get_package_share_directory(package_name),'urdf','bot_mecanum.urdf')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Get the models path for Gazebo resource path
    models_path = os.path.join(get_package_share_directory(package_name), 'models')
    
    # Add models path to GZ_SIM_RESOURCE_PATH
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Run the spawner node from the gazebo_ros package. 
    spawn_bot = Node(
                        package='ros_gz_sim', 
                        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'bot',
                   '-z', '0.2'],
                        output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )
    
    # Launch the Gazebo-ROS bridge for image topics
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/image"],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Launch Rviz with diff bot rviz file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',
                    parameters=[{'use_sim_time': True}])]
    )

    loc_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )

    # Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            'map': os.path.join(get_package_share_directory(package_name), 'config', 'maps.yaml'),
            'use_composition': 'True',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_rviz,
        declare_world,
        declare_use_sim_time,
        declare_nav2,

        # Launch the nodes
        rviz2,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        image_bridge,
        spawn_bot,
        loc_node,
        nav2_launch,
    ])
