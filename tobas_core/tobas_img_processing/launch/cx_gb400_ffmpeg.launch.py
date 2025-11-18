from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable,\
                           IncludeLaunchDescription,\
                           ExecuteProcess,\
                           DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare

# launch : ros2 launch tobas_img_processing cx_gb400_ffmpeg.launch.py server_and_port:=127.0.0.1:8888
# image receive command : ffplay -fflags nobuffer -autoexit "srt://127.0.0.1:8888?mode=listener"

# cx_gb400から映像を取得してffmpegを用いてsrt通信で送信しながら，ジンバルの制御も行う．ros topicでの映像の配信は行わない．
def generate_launch_description():
    server_and_port_arg = DeclareLaunchArgument(
        'server_and_port',
        default_value='127.0.0.1:8888',
        description='IP address and port of the image receiver'
    )
    image_scale_arg = DeclareLaunchArgument(
        'image_scale',
        default_value='2',
        description='Scale the image by this value. If set to 2, the published image will have half the original width and height.'
    )

    server_and_port = LaunchConfiguration('server_and_port')
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
            server_and_port,
            TextSubstitution(text='?pkt_size=1316"')
        ]
    ]

    retry_ffmpeg_cmd = [
        [
            TextSubstitution(text='while true; do\n'),
            *ffmpeg_cmd[0],
            TextSubstitution(text='\n echo "ffmpeg exited. Retrying in 5 seconds..."\n'),
            TextSubstitution(text='sleep 5\n'),
            TextSubstitution(text='done')
        ]
    ]

    ffmpeg_process = ExecuteProcess(
        name="ffmpeg_process",
        cmd=retry_ffmpeg_cmd,
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        server_and_port_arg,
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
