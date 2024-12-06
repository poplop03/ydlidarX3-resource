from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ms200_scan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('oradar_lidar'), 'launch'),
         '/ms200_scan_gmapping.launch.py'])
      )

    # yahboomcar_bringup_launch.py  rtabmap_bringup_launch.py
    bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_bringup'), 'launch'),
        '/yahboomcar_bringup_launch.py'])
    )


    launch_description = LaunchDescription([
        ms200_scan_node,
        bringup_node
        ]) 
    return launch_description
