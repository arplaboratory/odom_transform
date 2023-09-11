# Launch file to launch the odom transform node
# Importing all the required packages
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Defining launch arguments
    launch_args = [
        DeclareLaunchArgument(name="namespace", default_value="race1", description="namespace"),
    ]

    #transform_config_path = os.path.join(
    #    get_package_share_directory('odom_transform_ros2'),
    #    'config', 'transform.yaml')
    
    # Loading the odom transform composable node
    odom_transform_node = ComposableNode(
        #namespace=LaunchConfiguration("namespace"),
        package='odom_transform_ros2',
        plugin='transform_nodelet_ns::OvtransformNodeletClass',
        name='odom_transform_node',
        parameters=[{'transform_config_path' : '/home/yash/ros2_ws/src/odom_transform_ros2/config/transform.yaml'}],
        #remappings = [('~/odomimu', '/race1/odom_transform_nodelet/odomimu')]
    )

    # Adding the odom transform node to the existing container
    load_composable_nodes = LoadComposableNodes(
        target_container='my_container',
        composable_node_descriptions=[
            odom_transform_node
        ]
    )

    # Creating a launch file object and calling it
    ld = LaunchDescription(launch_args)
    ld.add_action(load_composable_nodes)

    return ld
