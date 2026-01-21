from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')
    
    # パッケージ共有ディレクトリ取得
    pkg_dir = get_package_share_directory('lsa_nav_bringup')
    nav_pkg = get_package_share_directory('lsa_nav_controller')

    # パラメータファイルパス
    lsa_nav_ctr_param = os.path.join(nav_pkg, 'param', 'param.controller.yaml')
    
    load_nodes = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'lsa_nav_wo_camera.launch.py')
                ), 
                launch_arguments={'use_rviz': use_rviz}.items()
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package='lsa_nav_controller',
                        executable='lsa_nav_controller',
                        name='lsa_nav_controller',
                        output='screen',
                        parameters=[lsa_nav_ctr_param], 
                    )
                ]
            )
        ])
    ld = LaunchDescription()
    ld.add_action(declare_arg_use_rviz)
    ld.add_action(load_nodes)
    return ld
