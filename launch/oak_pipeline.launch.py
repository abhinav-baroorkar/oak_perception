from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_info_topic', default_value='/rgb/camera_info'),
        DeclareLaunchArgument('depth_topic', default_value='/stereo/depth'),
        DeclareLaunchArgument('detections_2d_topic', default_value='/detections_2d'),

        Node(
            package='oak_perception',
            executable='depth_localizer',
            name='depth_localizer',
            output='screen',
            parameters=[{
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'detections_2d_topic': LaunchConfiguration('detections_2d_topic'),
            }]
        ),
    ])
