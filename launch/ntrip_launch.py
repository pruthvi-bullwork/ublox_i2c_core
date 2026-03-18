from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # NTRIP client parameters
    params = [{
        'use_https': LaunchConfiguration('use_https', default='false'),
        'host': LaunchConfiguration('host', default='caster.emlid.com'),
        'port': LaunchConfiguration('port', default='2101'),
        'mountpoint': LaunchConfiguration('mountpoint', default='MP23489'),
        'username': LaunchConfiguration('username', default='u94435'),
        'password': LaunchConfiguration('password', default='656kqp'),
        'log_level': LaunchConfiguration('log_level', default='INFO'),
        'maxage_conn': LaunchConfiguration('maxage_conn', default='30'),
    }]

    container_ntrip = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        respawn=True,
        respawn_delay=5.0,
        composable_node_descriptions=[]
    )

    ntrip_component = ComposableNode(
        package='ntrip_client_node',
        plugin='ublox_dgnss::NTRIPClientNode',
        name='ntrip_client',
        parameters=params,
    )

    loader_ntrip = LoadComposableNodes(
        target_container='/ntrip_client_container',
        composable_node_descriptions=[ntrip_component]
    )

    reload_on_start_ntrip = RegisterEventHandler(
        OnProcessStart(
            target_action=container_ntrip,
            on_start=[TimerAction(period=3.0, actions=[loader_ntrip])]
        )
    )

    return LaunchDescription([
        container_ntrip,
        reload_on_start_ntrip,
    ])
