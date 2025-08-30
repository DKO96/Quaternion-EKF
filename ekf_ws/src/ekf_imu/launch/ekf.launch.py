from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    px4_offboard = Node(
        package="ekf_imu",
        executable="px4_offboard",
        name="offboard_control"
    )

    estimate = Node(
        package="ekf_imu",
        executable="estimate",
        name="estimate"
    )

    ld = LaunchDescription()
    ld.add_action(px4_offboard)
    ld.add_action(estimate)

    return ld