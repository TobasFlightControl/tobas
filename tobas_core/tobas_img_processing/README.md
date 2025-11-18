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
    - disable_video_streaming : このnodeが映像をカメラから取得するか，否か．例えば，ffmpegの機能を使ってros2 topicではなく他の通信形式で送信したいときにはtrueにする．
    - image_topic : このnodeがpublishする映像のros topic名．
    - FPS : frames per second．このnodeが何Hzで回るかを指定する．結局カメラのFPSに相当する．
