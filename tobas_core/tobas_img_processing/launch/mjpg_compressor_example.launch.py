from launch import LaunchDescription
from launch_ros.actions import Node


# MJPGの映像を/dev/video0から取得してros topicとしてpublishし，それをH.264に圧縮し，それを解凍する例．解凍後の映像はrviz2で視聴可能．
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tobas_img_processing",
                executable="video_dev_publisher",
                parameters=[
                    {"use_compressed_image": True, "device_name": "/dev/video0", "image_topic": "image_compressed"}
                ],
                output="screen",
            ),
            Node(
                package="tobas_img_processing",
                executable="mjpg_compressor",
                parameters=[
                    {"mjpg_topic": "image_compressed", "resized_topic": "image_compressed_resized", "encoding": "H.264"}
                ],
                output="screen",
            ),
            Node(
                package="tobas_img_processing",
                executable="h264_decompressor",
                parameters=[
                    {"h264_topic": "image_compressed_resized", "decoded_topic": "image_compressed_resized/decoded"}
                ],
                output="screen",
            ),
        ]
    )
