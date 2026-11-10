"""Run the ROS 2 bridge against a physical MSP controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = get_package_share_directory('multiwii')
    node = Node(
        package='multiwii', executable='multiwii_node', name='multiwii', output='screen',
        parameters=[LaunchConfiguration('config'), {
            'device': LaunchConfiguration('device'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'cfg', 'sbc.yaml')),
        RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason='Driver exited'))])),
        node,
    ])
