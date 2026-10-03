"""Launch the R1 topic bridge with optional typed parameter overrides."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_STRING_PARAMETERS = (
    "topic_name",
    "msg_type",
    "server_name",
    "csm_name",
    "master_name",
    "channel_name",
    "controller_name",
    "csm_mode",
)
_INT_PARAMETERS = (
    "priority",
    "timeout_ms",
    "disconnect_timeout_ms",
    "csm_status_timer_interval_ms",
    "input_timeout_ms",
    "registration_timeout_ms",
    "registration_retry_ms",
)


def launch_setup(context):
    """Preserve YAML values unless an argument explicitly overrides them."""
    overrides = {}
    for name in _STRING_PARAMETERS + _INT_PARAMETERS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = (
                int(value)
                if name in _INT_PARAMETERS
                else ParameterValue(value, value_type=str)
            )
    return [
        ComposableNodeContainer(
            name="topic_bridge_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="rv2_csm_topic_bridge",
                    plugin="rv2_csm_topic_bridge::TopicBridgeNode",
                    name="topic_bridge",
                    parameters=[
                        LaunchConfiguration("config_file").perform(context),
                        overrides,
                    ],
                )
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    """Declare bridge settings and load the component in a spinning executor."""
    args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("rv2_csm_topic_bridge"),
                    "config",
                    "topic_bridge.yaml",
                ]
            ),
            description="R1 bridge ROS parameter YAML file",
        )
    ]
    args.extend(
        DeclareLaunchArgument(name, default_value="", description=f"Override {name}")
        for name in _STRING_PARAMETERS + _INT_PARAMETERS
    )
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
