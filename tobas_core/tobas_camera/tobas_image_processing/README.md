# tobas_image_processing

## nodes

- compressed_image_viewer
  - Displays MJPG video received from a ROS topic.
  - parameters
    - compressed_image_topic : Name of the ROS topic to receive and display.
- cx_gb400_publisher
  - Operates the cx_gb400.
  - Subscribes to the aircraft attitude and sends it to the camera.
  - Subscribes to the target attitude for the camera gimbal and sends it to the camera.
  - Publishes captured video to a ROS topic when `disable_video_streaming = false`.
  - parameters
    - device_name :
      Device name used by the OS to recognize the cx_gb400.
    - disable_video_streaming :
      Whether this node retrieves video from the camera. Set this to `true`,
      for example, when using ffmpeg to send video through a non-ROS transport.
    - image_topic :
      Name of the ROS topic where this node publishes video.
    - FPS :
      Frames per second. Specifies the operating frequency of this node, which effectively corresponds to the camera FPS.
- ffmpeg_to_ros_msg_converter
  - Receives video sent by ffmpeg or another non-ROS transport and republishes it as a ROS topic.
  - parameters
    - ros_image_topic : Topic name used when republishing.
    - protocol : Video transport protocol to receive.
    - port_uri : Port used to receive video.
    - output_msg_encoding : Encoding of the video when republishing.
    - frame_id : `frame_id` of the ROS video topic when republishing.
    - FPS : Frames per second of the incoming video.
- h264_decompressor
  - Subscribes to H.264-compressed video as `ffmpeg_image_transport_msgs/msg/FFMPEGPacket`, decompresses it, and publishes it.
  - parameters
    - h264_topic : Topic name for H.264-compressed video data.
    - decoded_topic : Topic name for decompressed video data.
- mjpg_decompressor
  - Subscribes to images of type `sensor_msgs::msg::CompressedImage` (equivalent to MJPG),
    decompresses them, and publishes them as `sensor_msgs::msg::Image`.
  - parameters
    - mjpg_topic : Topic name for MJPG-compressed video data.
    - decoded_topic : Topic name for decompressed video data.
- mjpg_compressor
  - Subscribes to images of type `sensor_msgs::msg::CompressedImage` (equivalent to MJPG),
    compresses them, and publishes them as MJPG or H.264 video.
  - parameters
    - mjpg_topic : Topic name for the MJPG input to be compressed.
    - resized_topic : Topic name after compression.
    - encoding : Encoding of the compressed video: MJPG or H.264.
    - resize_rate : Ratio used to reduce image width and height before compression when further compression is desired.
- video_dev_publisher
  - Retrieves video data from the camera and publishes it as a ROS topic.
  - parameters
    - use_compressed_image : Whether the video data retrieved from the camera is MJPG or an uncompressed format such as YUYV.
    - device_name : Device name used by the OS to recognize the camera device.
    - image_topic : Name of the ROS topic to publish.

## Displaying Video

Use rviz2 to display video.
Start rviz2, then add an Image panel from the Add button at the lower left.
Select the video topic name, then adjust QoS by setting Reliability to Best Effort.

However, the video topic type must be `sensor_msgs/msg/Image`.
In other words, the image must be uncompressed.
To display the contents of an MJPG or H.264 video topic, decompress it with another node and display the decompressed video topic in rviz2.
MJPG can be decompressed with `mjpg_decompressor`, and H.264 can be decompressed with `h264_decompressor`.

## launch files

- cx_gb400_ffmpeg.launch.py
  - Retrieves video from the cx_gb400 and sends it over SRT with ffmpeg while also controlling the gimbal.
    Video is not distributed as a ROS topic.
  - Example execution:
    ```bash
    $ ros2 launch tobas_image_processing cx_gb400_ffmpeg.launch.py server_and_port:=127.0.0.1:8888 # Send video.
    $ ffplay -fflags nobuffer -autoexit "srt://127.0.0.1:8888?mode=listener" # Receive video.
    ```
  - To republish it as a ROS topic and display it in rviz2, run:
    ```bash
    $ ros2 launch tobas_image_processing cx_gb400_ffmpeg.launch.py server_and_port:=127.0.0.1:8888 # Send video.
    $ ros2 run tobas_image_processing ffmpeg_to_ros_msg_converter
    $ rviz2 # Display the image topic as an image.
    ```
  - parameters
    - server_and_port :
      Destination server IP address and port number. Specify it as \<ip_address\>:\<port\>.
    - image_scale :
      Factor by which to reduce the width and height of the transmitted video. Setting this to 2 sends the video at half size.
- mjpg_compressor_example.launch.py
  - Example that retrieves MJPG video from `/dev/video0`, publishes it as a ROS topic, compresses it to H.264, and then decompresses it.
    The decompressed video can be viewed in rviz2.
