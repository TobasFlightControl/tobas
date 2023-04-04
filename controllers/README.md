# dh_multirotor/controllers

## ROS API

---

### Subscribed Topics

- /multirotor_controller/command (multirotor_msgs/Command) \
  ドローンの位置姿勢とその時間微分の目標値
- /<drone_name>/base_state (multirotor_msgs/PoseVelStamped) \
  ドローンの位置姿勢とその時間微分
- /<drone_name>/joint_states (sensor_msgs/JointState) \
  駆動関節の状態

### Published Topics

- /<drone_name>/command/motor_speed (multirotor_msgs/RotorSpeeds) \
  各モータの回転速度
