from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import launch

from launch.actions import TimerAction


def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888 -v '
        ]],
        shell=True
    )

    camera_depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[
                ('/camera', '/camera/image_raw'),
            ]
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        remappings=[
                ('/camera_info', '/camera/camera_info'),
            ]
    )

    aruco_node = Node(
        package="aruco_ros",
        namespace="camera",
        executable="marker_publisher",
        output="both",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.20,
            "camera_frame": "camera_link",
            "marker_frame": "marker",
        }],
        remappings=[
                ('/image', 'image_raw'),
                ('/camera_info', 'camera_info'),
            ]
    )

    offb_node = Node(
        package='rsp_quad',
        executable='offboard_control.py',
        output='screen',
        shell=True,
    )

    return launch.LaunchDescription(
        [
         micro_ros_agent,
         camera_depth_bridge,
         camera_bridge,
         camera_info_bridge,
         aruco_node,
         offb_node
        ]
    )