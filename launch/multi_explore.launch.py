from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def launch_tare_node(context: LaunchContext, scenario, robot_id):
    scenario_str = context.perform_substitution(scenario)
    id_str = context.perform_substitution(robot_id)
    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static'),
        # ],
        parameters=[get_package_share_directory('tare_planner') + '/' + scenario_str + '_' + id_str + '.yaml']
    )
    return [tare_planner_node]

def launch_rviz_node(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_rviz',
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static'),
        # ],
        arguments=[
            '-d', get_package_share_directory('tare_planner') + '/multi_' + id_str + '.rviz'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    return [rviz_node]

def push_namespace(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    return [PushRosNamespace('robot_' + str(id_str))]

def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    scenario = LaunchConfiguration('scenario')

    declare_scenario = DeclareLaunchArgument(
        'scenario',
        default_value='multi',
        description='description for scenario argument'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='description for rviz argument'
    )
    
    robot_id = LaunchConfiguration('robot_id')

    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )

    return LaunchDescription([
        # OpaqueFunction(function=push_namespace, args=[robot_id]),
        declare_use_sim_time_cmd,
        declare_scenario,
        declare_rviz,
        declare_robot_id,
        OpaqueFunction(function=launch_rviz_node, args=[robot_id]),
        OpaqueFunction(function=launch_tare_node, args=[scenario, robot_id])
    ])
