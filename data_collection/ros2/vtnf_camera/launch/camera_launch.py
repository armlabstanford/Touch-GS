from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vtnf_camera',
            executable='cam_pub',
            name='webcam_node',
            parameters=[{'camera_id': '/dev/webcam_1'}],
            output='screen'

        ),
        Node(
            package='vtnf_camera',
            executable='dtv2_cam_pub',
            name='dtv2_node',
            parameters=[{'camera_id': '/dev/dtv2_sen2'}],
            output='screen'

        ),
        
        
        
        #### ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090
        ExecuteProcess(
            cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml', 'port:=8765'],
            # output='screen'
        )
    ])