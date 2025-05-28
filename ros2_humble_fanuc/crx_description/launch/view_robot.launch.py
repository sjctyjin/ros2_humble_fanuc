from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription  # Correct import method
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="crx10ia_l",
            description="model of the fanuc robot. ",
        )
    )

    return LaunchDescription( declared_arguments + [OpaqueFunction(function=launch_setup)] )

def launch_setup(context, *args, **kwargs):
    
    robot_type = LaunchConfiguration("robot_type")
    robot_type_str = robot_type.perform(context)
    print("robot_type", robot_type_str)

    description_package = "crx_description"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf/"+robot_type_str+"/", robot_type_str+".xacro"]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}    
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "config", "config.rviz"])

    #GUI

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    cus_state_publisher_node = Node(
        package="fanuc_joint_sql",
        executable="joint_gui_pub",
        name='joint_gui_pubs',
        output="both",
    )
    robot_controller_node = Node(
        package="fanuc_joint_sql",
        executable="joint_states_sql",
        name='joint_states_sql',
        output="screen",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    
    realsense_launch_dir = os.path.join(
        FindPackageShare('realsense2_camera').find('realsense2_camera'),
        'launch',
    )
    
    cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            realsense_launch_dir, '/rs_launch.py'
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items()
    )
    
    cam1_yolo = Node(
        package='transform_example',
        executable='yolov8_detect',
        name='cam1_yolo',
        output='screen'
    )

    nodes_to_start = [
        #joint_state_publisher_node,
        robot_controller_node,
        cus_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        cam_node,
        cam1_yolo
    ]

    return nodes_to_start
