import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # 2) 啟動 static_transform_publisher (將相機座標轉換到 fanuc_world)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cam_to_fanuc',
        arguments=['1.0', '0.0', '0.7', '3.14', '0', '0', 'fanuc_world', 'camera_link']
    )

    # 3) 啟動 fanuc_joint_sql 的 moveit_fram_to_world
    #    假設在您的 setup.py 中，console_scripts 定義為
    #    'moveit_fram_to_world = fanuc_joint_sql.moveit_fram_to_world:main'
    fram_to_world = Node(
        package='fanuc_joint_sql',
        executable='moveit_fram_to_world',
        name='moveit_fram_to_world_node'
    )

    # 4) 啟動 fanuc_joint_sql 的 moveit_goal_setting
    #    同樣假設 console_scripts 有
    #    'moveit_goal_setting = fanuc_joint_sql.moveit_goal_setting:main'
    # goal_setting = Node(
    #     package='fanuc_joint_sql',
    #     executable='moveit_goal_setting',
    #     name='moveit_goal_setting_node'
    # )

    # 組合 LaunchDescription
    ld = LaunchDescription()

    # 將上述 actions/包含檔加進來
    # ld.add_action(moveit_demo)
    ld.add_action(static_tf_pub)
    ld.add_action(fram_to_world)
    # ld.add_action(goal_setting)

    return ld
