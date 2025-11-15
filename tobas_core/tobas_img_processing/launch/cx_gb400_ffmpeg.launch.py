from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable,\
                           IncludeLaunchDescription,\
                           ExecuteProcess,\
                           DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare

# launch : ros2 launch tobas_img_processing cx_gb400_ffmpeg.launch.py ip_addr:=127.0.0.1
# image receive command : ffplay -fflags nobuffer -autoexit "srt://127.0.0.1:8888?mode=listener"
# caution : receive command must be executed earlier

def generate_launch_description():
    ip_addr_arg = DeclareLaunchArgument(
        'ip_addr',
        default_value='127.0.0.1',
        description='IP address of the image receiver'
    )
    image_scale_arg = DeclareLaunchArgument(
        'image_scale',
        default_value='2',
        description='Scale the image by this value. If set to 2, the published image will have half the original width and height.'
    )

    ip_addr = LaunchConfiguration('ip_addr')
    image_scale = LaunchConfiguration('image_scale')

    ffmpeg_cmd = [
        [
            TextSubstitution(text='ffmpeg -f v4l2 -video_size 1280x720 -input_format mjpeg -i /dev/video0 '),
            TextSubstitution(text='-r 15 -c:v libx264 '),
            TextSubstitution(text='-vf scale=iw/'),
            image_scale,
            TextSubstitution(text=':-1 '),
            TextSubstitution(text='-tune zerolatency -preset faster -f mpegts '),
            TextSubstitution(text='"srt://'),
            ip_addr,
            TextSubstitution(text=':8888?pkt_size=1316"')
        ]
    ]

    ffmpeg_process = ExecuteProcess(
        cmd=ffmpeg_cmd,
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        ip_addr_arg,
        image_scale_arg,
        Node(
            package='tobas_img_processing',
            executable='cx_gb400_publisher',
            parameters = [
                {"device_name" : "/dev/video0",
                "disable_video_streaming" : True}
            ],
            output='screen'
        ),
        ffmpeg_process,
    ])
