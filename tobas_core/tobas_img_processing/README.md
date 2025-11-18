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
- mjpg_decompressor
  - sensor_msgs::msg::CompressedImage型（MJPGに相当）の画像をsubscribeし，解凍して，sensor_msgs::msg::Image型としてpublishする．
  - parameters
    - mjpg_topic : mjpgに圧縮された映像データのtopic名．
    - decoded_topic : 解凍された映像データのtopic名．
- mjpg_compressor
  - sensor_msgs::msg::CompressedImage型（MJPGに相当）の画像をsubscribeし，圧縮して，MJPGかH.264の動画としてpublishする．
  - parameters
    - mjpg_topic : MJPG形式の圧縮予定のtopic名．
    - resized_topic : 圧縮後のtopic名．
    - encoding : 圧縮後の動画のencoding．MJPGかH.264．
    - resize_rate : 画像を圧縮する前に画像の幅・高さをこの比率で縮めることにより，さらに圧縮率を上げる場合に設定する．
- video_dev_publisher
  - カメラから映像データを取得し，ros topicとしてpublishする．
  - parameters
    - use_compressed_image : カメラから取得する映像データがMJPG形式なのかYUYV等の非圧縮形式なのか．
    - device_name : カメラデバイスがOSにどの名前で認識されているか．
    - image_topic : publishするros topic名．


## 映像の描画について

映像の描画のためにはrviz2を用いる．
rviz2を立ち上げ，左下のAddボタンからImageというパネルを追加する．
そして，映像topic名を選択し，さらにQoSを調整して（ReliablityをBest Effortに）表示する．

ただし，この映像topicの型はsensor_msgs/msg/Image型である必要がある．
すなわち圧縮されていない画像である必要がある．
そのため，MJPGやH.264の映像topicの中身を表示する場合には，
別のnodeにより解凍し，解凍後の映像topicをrviz2により表示する必要がある．
MJPGであればmjpg_decompressorにより，H.264であればh264_decompressorにより解凍することができる．

