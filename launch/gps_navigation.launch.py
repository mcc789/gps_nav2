"""
启动GPS导航和gazebo仿真环境
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    全局参数配置
    """
    pkg_name = 'gps_nav2' 
    pkg_share = get_package_share_directory(pkg_name)
    
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_gps_params.yaml')  # Nav2导航参数
    ekf_params_file = os.path.join(pkg_share, 'config', 'dual_ekf_navsat.yaml')  # 双EKF+GPS转换参数
    world_path = os.path.join(pkg_share, 'world', 'field.world')  # Gazebo仿真场景文件
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')  # 机器人模型文件
    
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')  # rviz2配置


    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )

    # 生成机器人
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'fishbot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen',
    )
    
    # 机器人状态发布
    robot_desc = xacro.process_file(urdf_path).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # 加载关节状态广播器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 加载差速驱动控制器
    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fishbot_diff_drive_controller"],
        output="screen",
    )

    # 局部EKF定位节点
    robot_localization_odom = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        output='screen'
    )
    
    # 全局EKF定位节点
    robot_localization_map = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_map',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')],  # 话题重映射
        output='screen'
    )

    # GPS坐标转换节点
    navsat_transform = Node(
        package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[
            ('imu/data', 'imu'), 
            ('gps/fix', 'gps/fix'), 
            ('odometry/filtered', 'odometry/gps') 
        ],
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 启动Nav2导航框架
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
            'map_subscribe_transient_local': 'True'
        }.items(),
    )

    # 启动rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
        # 延时加载控制器，防止 Gazebo 没准备好
        TimerAction(period=3.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[load_diff_drive_controller]),
        robot_localization_odom,
        robot_localization_map,
        navsat_transform,
        nav2_bringup,
        rviz_node
    ])



# ros2 launch gps_nav2 gps_navigation.launch.py