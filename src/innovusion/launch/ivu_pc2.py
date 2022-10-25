from launch import LaunchDescription
from launch_ros.actions import Node

# from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # para_dir = os.path.join(get_package_share_directory('localizer'), 'config', 'localizer.yaml')
    return LaunchDescription(
        [
            Node(
                package="innovusion",
                executable="subscriber",
            ),
            Node(
                package="innovusion",
                executable="publisher",
                parameters=[
                    {"lidar_name": "falcon"},
                    {"frame_id": "map"},
                    {"lidar_ip": "172.168.1.10"}, # Lower priority than data_filename
                    {"lidar_port":8001},
                    {"lidar_model": "r"},
                    {"reflectance": 1},
                    {"multireturn": 1},
                    {
                        "data_filename": "" # support *.inno_raw file for playback, If set, lidar_ip will be invalid
                    },
                    {"yaml_filename": ""},
                    {"file_speed": 10000}, # 0 means play as fast as possible, e.g, 15000 means play at 1.5x speed
                    {"file_rewind": -1}, # 0 means no rewind, < 0 means rewind infinity times.
                    {"file_skip": 0}, # skip * bytes of inno_raw file
                    {"lidar_udp_port": 0}, # recv data from udp port(like 8010), or use tcp as default(0)
                    {"processed": 0}, # 0 means process raw signal from lidar/inno_raw, 1 means process structured data from lidar
                    {"set_falcon_eye": False},
                    {"roi_center_h": 0},
                    {"roi_center_v": 0},
                    {"inno_log_level": 2},
                ],
            ),
        ]
    )
