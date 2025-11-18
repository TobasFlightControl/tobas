# tobas_img_processing

## nodes

- compressed_image_viewer
  - ros topicで送られてきたMJPG形式の動画を描画する．
  - parameters
    - compressed_image_topic : 受け取って描画するros topic名．
- cx_gb400_publisher
  - cx_gb400を動かす．
  - 機体姿勢をsubscribeし，カメラに送る．
  - カメラのジンバルの目標姿勢をsubscribeし，カメラに送る．
  - 撮影された映像をros topicにpublishする(disable_video_streaming = falseの場合)．
  - parameters
    - device_name : cx_gb400がOSにどの名前で認識されているか．
    - disable_video_streaming : このnodeが映像をカメラから取得するか，否か．例えば，ffmpegの機能を使ってros topicではなく他の通信形式で送信したいときにはtrueにする．
    - image_topic : このnodeがpublishする映像のros topic名．
    - FPS : frames per second．このnodeが何Hzで回るかを指定する．結局カメラのFPSに相当する．
- ffmpeg_to_ros_msg_converter
  - ffmpeg等からros topicではない通信形式で映像が送られてきた際に，その映像を受信し，ros topicとして再びpublishしなおす．
  - parameters
    - ros_image_topic : 再publishする際のtopic名．
    - protocol : 受信する映像伝送のプロトコル．
    - port_uri : 映像を受信するポート．
    - output_msg_encoding : 再publishする際の動画のencoding．
    - frame_id : 再publishする際の動画のros topicのframe_id．
    - FPS : frames per second．送られてくる動画のFPS．
- h264_decompressor
  - ffmpeg_image_transport_msgs/msg/FFMPEGPacket型のh.264で圧縮された映像をsubscribeして，解凍してpublishする．
  - parameters
    - h264_topic : h.264に圧縮された映像データのtopic名．
    - decoded_topic : 解凍された映像データのtopic名．
