import os

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    launch_velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('velodyne_driver'), 'launch', 'velodyne_driver_node-VLP16-launch.py')
        )
    )
    
    launch_velodyne_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('velodyne_pointcloud'), 'launch', 'velodyne_transform_node-VLP16-launch.py')
        )
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("logistic_description"), "config", "velodyne.rviz"]
    )

    rviz_node = Node(
       package="rviz2",
       executable="rviz2",
       name="rviz2",
       output="log",
       arguments=["-d", rviz_config_file],
       condition=IfCondition(gui),
    )

    return LaunchDescription([

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'velodyne_points'),
                            ('scan', 'scan')],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.05,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.0174,
                'scan_time': 0.1,        
                'range_min': 0.45,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'),

        launch_velodyne,
        launch_velodyne_driver,
        rviz_node,
    ])