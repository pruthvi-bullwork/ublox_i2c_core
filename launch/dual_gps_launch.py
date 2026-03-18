import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Path to your existing NTRIP launch (the one you provided)
    # Assuming you saved it as ntrip_launch.py in the same folder
    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ublox_i2c_core'), 'launch', 'ntrip_launch.py')
        ])
    )

    return LaunchDescription([
        # NTRIP Client (Fetches from Internet)
        ntrip_launch,

        # I2C Manager (Bridges NTRIP topic -> I2C Hardware)
        Node(package='ublox_i2c_core', executable='i2c_manager_node', name='i2c_bridge'),

        # Base Node (Publishing /gps/fix)
        Node(package='ublox_i2c_core', executable='base_node', name='base_gps'),

        # Rover Node (Publishing /rover/fix and /imu_gps/data)
        Node(package='ublox_i2c_core', executable='rover_node', name='rover_gps')
    ])