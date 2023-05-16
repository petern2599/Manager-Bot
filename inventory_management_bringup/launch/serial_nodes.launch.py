from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    serial_motor_controller_node = Node(
        package="inventory_management",
        executable="serial_motor_controller"
    )

    serial_publisher_node = Node(
        package="inventory_management",
        executable="serial_publisher",
        parameters=[
            {"ax_offset": 0.42},
            {"ay_offset": 0.06},
            {"az_offset": -0.59},
            {"gx_offset": -0.05},
            {"gy_offset": -0.02},
            {"gz_offset": -0.02},
        ]
    )

    ld.add_action(serial_motor_controller_node)
    ld.add_action(serial_publisher_node)
    return ld