import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('snowbot123')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    aruco_enabled = LaunchConfiguration('aruco_enabled', default='true')

    # ──────────────────────────────────────────────
    # 1. Robot in Gazebo (RSP + bridges + EKF)
    # ──────────────────────────────────────────────
    world_path = os.path.join(pkg_share, 'worlds', 'wow.sdf')

    models_path = os.path.join(pkg_share, 'models')
    gz_resource_path = models_path + os.pathsep + os.path.dirname(pkg_share)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path + os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v1 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g '}.items()
    )

    spawn_bot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'bot', '-z', '0.2'],
        output='screen'
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}']
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/image"],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ]
    )

    # ──────────────────────────────────────────────
    # 2. SLAM Toolbox (online async mapping)
    # ──────────────────────────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml'),
        }.items()
    )

    # ──────────────────────────────────────────────
    # 3. Nav2 (without AMCL — SLAM provides map)
    # ──────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_slam_params.yaml'),
            'use_localization': 'False',
            'map': '',
        }.items()
    )

    # ──────────────────────────────────────────────
    # 4. RViz
    # ──────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # ──────────────────────────────────────────────
    # 5. ArUco marker detection + navigation
    # ──────────────────────────────────────────────
    aruco_node = Node(
        package='snowbot123',
        executable='aruco_follow',
        name='aruco_follow',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image',
            'camera_info_topic': '/camera/camera_info',
            'aruco_dict': 'DICT_4X4_50',
            'marker_size': 0.10,
            'stop_distance': 0.5,
            'spin_speed': 0.5,
            'linear_speed': 0.3,
            'total_markers': 4,
            'parent_frame': 'camera_link',
            'camera_matrix_use_override': False,
        }],
        condition=IfCondition(aruco_enabled)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('aruco_enabled', default_value='true'),

        rsp,
        gazebo_server,
        gazebo_client,
        spawn_bot,
        ros_gz_bridge,
        image_bridge,
        ekf_node,
        slam_toolbox,
        nav2,
        aruco_node,
        rviz_node,
    ])
