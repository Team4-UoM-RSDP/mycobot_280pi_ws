from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycobot_280pi_adaptive_gripper_moveit2_control',
            executable='sync_plan_pi',
            name='sync_plan_pi',
            output='screen',
            parameters=[
                {'port': '/dev/serial0'},  # Raspberry Pi typical serial
                {'baud': 1000000}
            ]
        )
    ])
