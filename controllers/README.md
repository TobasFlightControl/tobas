# tobas/controllers

## ROS API

---

### Subscribed Topics

- /(drone_name)/command/base_state (tobas_msgs/Command) \
  ドローンの位置姿勢とその時間微分の目標値
- /(drone_name)/base_state (tobas_msgs/BaseState) \
  ドローンの位置姿勢とその時間微分
- /(drone_name)/joint_states (sensor_msgs/JointState) \
  駆動関節の状態

### Published Topics

- /(drone_name)/command/motor_speed (tobas_msgs/RotorSpeeds) \
  各モータの回転速度
