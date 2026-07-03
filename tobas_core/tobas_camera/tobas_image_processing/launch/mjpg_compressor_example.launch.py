from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Example pipeline that captures MJPG video from /dev/video0, publishes it as a ROS topic,
    compresses it to H.264, and then decompresses it.
    The decompressed video can be viewed in rviz2.
    """
    return LaunchDescription(
        [
            Node(
                package="tobas_image_processing",
                executable="video_dev_publisher",
                parameters=[
                    {
                        "use_compressed_image": True,
                        "device_name": "/dev/video0",
                        "image_topic": "image_compressed",
                    }
                ],
                output="screen",
            ),
            Node(
                package="tobas_image_processing",
                executable="mjpg_compressor",
                parameters=[
                    {
                        "mjpg_topic": "image_compressed",
                        "resized_topic": "image_compressed_resized",
                        "encoding": "H.264",
                    }
                ],
                output="screen",
            ),
            Node(
                package="tobas_image_processing",
                executable="h264_decompressor",
                parameters=[
                    {
                        "h264_topic": "image_compressed_resized",
                        "decoded_topic": "image_compressed_resized/decoded",
                    }
                ],
                output="screen",
            ),
        ]
    )
