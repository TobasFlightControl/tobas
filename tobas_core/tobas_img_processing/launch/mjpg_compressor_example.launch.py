from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tobas_img_processing',
            executable='video_dev_publisher',
            parameters = [
                {"use_compressed_image" : True,
                "device_name" : "/dev/video0",
                "compressed_image_topic" : "image_compressed"}
            ],
            output='screen'
        ),
        Node(
            package='tobas_img_processing',
            executable='mjpg_compressor',
            parameters = [
                {"mjpg_topic" : "image_compressed",
                 "resized_topic" : "image_compressed_resized",
                 "encoding" : "H.264"
                }
            ],
            output='screen'
        ),
        Node(
            package='tobas_img_processing',
            executable='h264_decompressor',
            parameters = [
                {"h264_topic" : "image_compressed_resized",
                 "decoded_topic" : "image_compressed_resized/decoded"}
            ],
            output='screen'
        ),
    ])
