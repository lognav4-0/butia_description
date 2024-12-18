import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

'''
We added this launch here to be easier to find and change de config files.

TODO: Velodyne URDF available here.
'''


def generate_launch_description():
    # Declare argument for RViz
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_rviz = LaunchConfiguration("use_rviz")

    config_dir = ament_index_python.packages.get_package_share_directory('sensors_description')

    # Velodyne driver node
    driver_params_file = os.path.join(config_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    # Velodyne transform node
    convert_params_file = os.path.join(config_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(config_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params])

    # Velodyne laserscan node
    laserscan_params_file = os.path.join(config_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])

    # Initialize RViz node if use_rviz is true
    rviz_config_file = os.path.join(config_dir, 'rviz', 'velodyne.rviz')  # Path to your RViz config file
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # Return LaunchDescription
    return LaunchDescription(
        declared_arguments + [
            velodyne_driver_node,
            velodyne_transform_node,
            velodyne_laserscan_node,
            rviz_node,  # Add RViz node conditionally
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=velodyne_driver_node,
                    on_exit=[launch.actions.EmitEvent(
                        event=launch.events.Shutdown())],
                )),
        ]
    )
