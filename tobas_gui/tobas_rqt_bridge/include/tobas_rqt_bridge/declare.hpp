#pragma once

#include <QMetaType>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/imu_stamped.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/magnetic_field_stamped.hpp>
#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_msgs/msg/post_arm_check.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/sbus.hpp>

// tobas_ros_interfaceで扱うトピックのうち，FCからPCに流れるもの
Q_DECLARE_METATYPE(tobas_msgs::msg::Battery::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::EngineState::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::Cpu::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::Sbus::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::RCInput::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::Gnss::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::RotorStateArray::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::JointStateArray::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::Odometry::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::Arming::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::PreArmCheck::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::PostArmCheck::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::ImuStamped::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::MagneticFieldStamped::ConstSharedPtr);
Q_DECLARE_METATYPE(tobas_msgs::msg::RosbagState::ConstSharedPtr);
