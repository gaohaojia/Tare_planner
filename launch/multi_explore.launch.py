from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
import os

def launch_tare_node(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('tare_planner'), 'config', 'multi_' + id_str + '.yaml')]
    )
    return [tare_planner_node]

def launch_rviz_node(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_rviz',
        arguments=[
            '-d', os.path.join(get_package_share_directory('tare_planner'), 'rviz', 'multi_' + id_str + '.rviz')],
        output='screen',
    )
    return [rviz_node]

def push_namespace(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    return [PushRosNamespace('robot_' + str(id_str))]

def generate_launch_description():

    robot_id = LaunchConfiguration('robot_id')

    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )
    
    return LaunchDescription([
        declare_robot_id,
        OpaqueFunction(function=launch_rviz_node, args=[robot_id]),
        OpaqueFunction(function=launch_tare_node, args=[robot_id])
    ])
