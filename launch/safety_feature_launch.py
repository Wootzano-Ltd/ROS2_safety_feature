from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_feature',
            executable='safety_node',
            remappings=[
                ("joint_states", "joint_states"),
                ("required_torque", "required_torque"),
                ("digital_output", "digital_output"),
                ("analog_output", "analog_output")
            ],
            parameters=[
                {"planning_group": "my_robot"},
                {"analog_threshold": 10},
                {"torque_diff_threshold": [10, 10, 10, 10, 10, 10]}
            ]
        )
    ])
