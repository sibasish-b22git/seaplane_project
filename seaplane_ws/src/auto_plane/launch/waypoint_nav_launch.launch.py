import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. The Mission Commander (Your Python Script)
    autopilot_node = Node(
        package='auto_plane',
        executable='plane_waypoint_nav',
        name='plane_autopilot_node',
        output='screen'
    )

    # 2. The Vision Translator (Depth to LaserScan)
    vision_avoidance_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'output_frame': 'base_link',
            'range_min': 0.5,
            'range_max': 60.0,   # matches DANGER_DISTANCE in Python
            'scan_height': 10,   # number of pixel rows to use from depth image
        }],
        remappings=[
            ('depth',             '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/color/camera_info'),
            # FIX 1: output to /scan so Python code receives it
            # Do NOT send to /mavros/obstacle/send — that triggers
            # ArduPilot's own avoidance which conflicts with our custom logic
            ('scan',              '/scan'),
        ]
    )
    
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link_tf',
        parameters=[{'use_sim_time': True}],
        # Format: X Y Z Yaw Pitch Roll parent_frame child_frame
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'cessna_aircraft/base_link/front_depth_camera']
    )

    # Return the master launch description
    return LaunchDescription([
        autopilot_node,
        vision_avoidance_node,
        camera_tf_node
    ])
