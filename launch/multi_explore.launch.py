from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    start_tare_planner_node = Node(
        package="tare_planner",
        executable="tare_planner_node",
        name="tare_planner_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("tare_planner"),
                "config",
                "tare_planner.yaml",
            )
        ],
    )

    start_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="tare_planner_rviz",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("tare_planner"),
                "rviz",
                "multi_explore.rviz",
            ),
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(start_tare_planner_node)
    ld.add_action(start_rviz_node)

    return ld
