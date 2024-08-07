from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


CONFIG_FILENAME = "config.yaml"

def generate_launch_description():
  launch_description = LaunchDescription()
  configuration = os.path.join(get_package_share_directory('as_manager'), 'config', CONFIG_FILENAME)
  
  node = Node(
    package="as_manager",
    name="as_manager_node",
    executable="as_manager_node",
    output="screen",
    parameters=[config_node]
  )

  launch_description.add_action(node)
  return launch_description
