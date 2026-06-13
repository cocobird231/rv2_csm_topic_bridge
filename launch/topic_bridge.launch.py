"""
Launch file for the rv2_csm_topic_bridge package.

Starts TopicBridgeNode inside a multi-threaded composable-node container
(component_container_mt).  Node parameters are loaded from a YAML config file
and may be selectively overridden via launch arguments.

The node subscribes to a configurable ROS 2 topic and bridges incoming
messages into the rv2 ControlSignalManager (CSM) as a control signal source.

Launch arguments:
  config_file     - Path to the ROS 2 parameter YAML file.
                    Defaults to config/topic_bridge.yaml in the package.
  topic_name      - ROS 2 topic to subscribe to (overrides config file when set).
  msg_type        - Message type: "joy", "twist", or "string"
                    (overrides config file when set).
  server_name     - Target CSM server name (overrides config file when set).
  channel_name    - CSM channel name (overrides config file when set).

Example:
  ros2 launch rv2_csm_topic_bridge topic_bridge.launch.py \\
      topic_name:=/cmd_vel msg_type:=twist server_name:=control_server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    config_file  = LaunchConfiguration('config_file').perform(context)
    topic_name   = LaunchConfiguration('topic_name').perform(context)
    msg_type     = LaunchConfiguration('msg_type').perform(context)
    server_name  = LaunchConfiguration('server_name').perform(context)
    channel_name = LaunchConfiguration('channel_name').perform(context)

    # Start with the YAML file; only apply overrides for non-empty arguments
    # so config-file values are never silently clobbered by defaults.
    parameters = [config_file]
    overrides = {}
    if topic_name:
        overrides['topic_name'] = topic_name
    if msg_type:
        overrides['msg_type'] = msg_type
    if server_name:
        overrides['server_name'] = server_name
    if channel_name:
        overrides['channel_name'] = channel_name
    if overrides:
        parameters.append(overrides)

    container = ComposableNodeContainer(
        name='topic_bridge_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='rv2_csm_topic_bridge',
                plugin='rv2_csm_topic_bridge::TopicBridgeNode',
                name='topic_bridge',
                parameters=parameters,
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    # ── Launch argument declarations ──────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rv2_csm_topic_bridge'),
                'config',
                'topic_bridge.yaml',
            ]),
            description='Path to TopicBridgeNode ROS 2 parameter YAML file'),

        DeclareLaunchArgument(
            'topic_name',
            default_value='',
            description='ROS 2 topic to subscribe to '
                        '(overrides config file when non-empty)'),

        DeclareLaunchArgument(
            'msg_type',
            default_value='',
            description='Message type: "joy", "twist", or "string" '
                        '(overrides config file when non-empty)'),

        DeclareLaunchArgument(
            'server_name',
            default_value='',
            description='Target CSM server name '
                        '(overrides config file when non-empty)'),

        DeclareLaunchArgument(
            'channel_name',
            default_value='',
            description='CSM channel name '
                        '(overrides config file when non-empty)'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
