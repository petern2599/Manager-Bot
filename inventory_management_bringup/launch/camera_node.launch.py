from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera_node0 = Node(
        package="inventory_management",
        executable="image_publisher",
        name="image_publisher_node0",
        remappings=[
            ("video_frame","video_frame0")
        ],
        parameters=[
            {"camera_index": 0},
            {"resize_percent": 25}
        ]
    )

    camera_node1 = Node(
        package="inventory_management",
        executable="image_publisher",
        name="image_publisher_node1",
        remappings=[
            ("video_frame","video_frame1")
        ],
        parameters=[
            {"camera_index": 2},
            {"resize_percent": 25}
        ]
    )

    ld.add_action(camera_node0)
    # ld.add_action(camera_node1)
    return ld