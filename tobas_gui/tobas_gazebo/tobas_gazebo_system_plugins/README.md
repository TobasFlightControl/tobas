# tobas_gazebo_system_plugins

## hesai_lidar_plugin

### 仕様

Hesai社のLiDARを模擬するplugin．
Hesai社公式のROS2 driverにQoSやros topicの形式を合わせている．

LiDARの点群の歪み + 各点ごとのtimestampを再現するための実装として，
一例としてgazeboのgpu_ray pluginを使って100Hzで点群データを取得し，
そのデータ1つから全体の1/10個の点を選択して，
10回分を重ね合わせて10HzのデータとしてROS topicに再発行することにしている．

LiDARの仕様がよくわからないので，一旦以下のような仮決めを行って制作した．
128という数字はHesai JT 128を使用する際の一例である．
- LiDARの中には128個のレーザー発光体が垂直に並べてあって，これが回っている．メッセージで送られてくるringは発光体の番号で，一番下から0 ~ 127となっている．
- メッセージで送られてくるheaderのtimestampはフレームの最初の時間（Hesai driverのソースコードにそうっぽいことが書いてある），各点ごとに設定されるtimestampはその時刻からのズレの時間で単位は秒（怪しい）．
- 128段の発光体列は静止状態において同一方向を向いていない．まんべんなく全周方向に向いている．1段上に上がるごとに角度が360 / 128 degreeずつズレていく．

発行するデータはLiDAR点群データとLiDARに搭載されているimuのデータ．
topic名はtobas_constantsで管理してあり，
それぞれtobas::kLidarPointCloudTopicとtobas::kLidarImuTopicである．

### 使用方法

現在tobas_setup_assistantによるセットアップを実装していないため，
本pluginを使用する際は手動で設定を行う必要がある．
一例としてf450なる機体のTBS packageとしてtobas_f450.TBSを作成した場合を考える．
この場合，tobas_f450.TBS/tobas_f450_config/urdf/drone.xacroを編集する．
そして以下のようにコードを追加する．
```xml
<robot name="f450" xmlns:xacro="http://ros.org/wiki/xacro">
    <!-- 前略 -->
    <gazebo reference="fmu">
        <sensor name='HesaiJT128' type='gpu_lidar'>
            <pose>0 0 0.1 0 0 0</pose>
            <always_on>1</always_on>
            <visualize>1</visualize>
            <topic>f450/lidar</topic>
            <update_rate>100</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>900</samples>
                        <resolution>1</resolution>
                        <min_angle>0</min_angle>
                        <max_angle>6.283</max_angle>
                    </horizontal>
                    <vertical>
                        <samples>128</samples>
                        <resolution>1</resolution>
                        <min_angle>-0.0872</min_angle>
                        <max_angle>1.57</max_angle>
                    </vertical>
                </scan>
                <range>
                    <min>0.1</min>
                    <max>60.0</max>
                    <resolution>0.001</resolution>
                </range>
            </ray>
        </sensor>
    </gazebo>
    <gazebo>
        <plugin filename="tobas_gazebo_hesai_lidar_plugin" name="gazebo::HesaiLidarPlugin">
            <robotNamespace>f450</robotNamespace>
            <lidar>
                <gpuRayTopic>/f450/lidar/points</gpuRayTopic>
                <gpuRayUpdateRate>100</gpuRayUpdateRate>
                <updateRate>10</updateRate>
                <horizontalSamples>900</horizontalSamples>
                <verticalSamples>128</verticalSamples>
            </lidar>
            <imu>
                <linkName>base_link</linkName>
                <updateRate>400</updateRate>
                <offset>0 0 0</offset>
                <gyroNoiseDensity>0.00019198621771937625</gyroNoiseDensity>
                <gyroRandomWalk>0</gyroRandomWalk>
                <gyroBiasCorrelationTime>1000</gyroBiasCorrelationTime>
                <accelNoiseDensity>0.0016671305000000001</accelNoiseDensity>
                <accelRandomWalk>0</accelRandomWalk>
                <accelBiasCorrelationTime>300</accelBiasCorrelationTime>
            </imu>
        </plugin>
    </gazebo>
    <!-- 後略 -->
</robot>
```

最初の\<gazebo\>タグがgazebo公式のgpu_lidar pluginを利用する設定である．
変更可能性があるのは以下の部分．
- gazebo reference = "fmu", lidarをどのframeに設置するかを設定する．この場合はfmuというframe（フラコンに相当）に設置されている．
- \<pose\>0 0 0.1 0 0 0\</pose\>, 上で設定したframeのどの座標・回転の部分にlidarを設置するかを設定する．この場合はfmu frameから見てz軸方向に0.1mずらした場所に設置されている．
- \<topic\>f450/lidar\</topic\>, このgazebo pluginがpublishするlidar点群のgazebo topicのtopic名．名前衝突しなければ何でも良いがこの場合にはmachine_name/lidarとしている．
- \<update_rate\>100\</update_rate\>, lidarの歪みを模擬するために本来のlidarより高い周波数でupdateさせるために本来のlidarのupdate rate f_1よりも高い値f_2を設定する．f_2がf_1で割り切れることを推奨．
- \<ray\>\</ray\>内, lidarのFoVと解像度に合わせて設定する．

次の\<gazebo\>タグがtobas_gazebo_heasi_lidar_pluginを利用する設定である．
変更可能性があるのは以下の部分．
- \<robotNamespace\>f450\</robotNamespace\>, machine_nameの変更に伴い変更．
- \<lidar\>\<gpuRayTopic\>/f450/lidar/points\</gpuRayTopic\>\</lidar\>, gazeboのgpu_lidar pluginのところで\<topic\>として指定したものにpointsをつけたものに変更．
- \<lidar\>\<gpuRayUpdateRate\>/f450/lidar/points\</gpuRayUpdateRate\>\</lidar\>, gazeboのgpu_lidar pluginのところで\<update_rate\>として指定したものと一致させる．
- \<lidar\>\<updateRate\>10\</updateRate\>\</lidar\>, 本当のlidarのupdate rate f_1を指定する．
- \<lidar\>\<horizontalSamples\>900\</horizontalSamples\>\</lidar\>, gazebo gpu_lidar pluginのところで指定したものと一致させる．
- \<lidar\>\<verticalSamples\>128\</verticalSamples\>\</lidar\>, gazebo gpu_lidar pluginのところで指定したものと一致させる．
- \<imu\>\</imu\>以下, lidarに搭載されているimuのノイズの状況に応じて変更する．
