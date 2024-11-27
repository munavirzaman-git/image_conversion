import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='usb_cam',
            output='screen',
            remappings=[('/usb_cam/image_raw', '/usb_cam/image_raw')]
        ),
        Node(
            package='image_conversion',
            executable='image_conversion_node',
            name='image_conversion',
            output='screen'
        )
    ])

