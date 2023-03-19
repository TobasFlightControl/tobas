#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>

#include <dh_std_tools/buffer.hpp>
#include <multirotor_msgs/LinearVelocityWithCovarianceStamped.h>
#include <dh_kdl_msgs/PoseVel.h>

#include "./cartesian_filter.hpp"

class StateEstimator
{
  using ImuMsg = sensor_msgs::Imu;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = multirotor_msgs::LinearVelocityWithCovarianceStamped;

public:
  StateEstimator(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

  // rosparam
  const bool use_bar_;
  const bool use_gps_pos_;
  const bool use_gps_vel_;

  bool is_ready_;
  ros::Time t_last_;
  double lat_0_;                             // 緯度のゼロ点
  double lon_0_;                             // 経度のゼロ点
  double alt_0_;                             // 高度のゼロ点
  Eigen::Quaterniond quat_;                  // 推定された姿勢
  Eigen::Vector2d xy_m_;                     // 絶対平面位置の測定値 (world)
  Eigen::Vector3d v_m_;                      // 絶対速度の測定値 (world)
  Eigen::Vector3d a_m_;                      // 加速度の観測値 (local)
  dh_std::Buffer<ImuMsg> filtered_imu_buf_;  // 姿勢推定済みのIMUデータ
  dh_std::Buffer<BarMsg> bar_buf_;           // 気圧センサの観測値
  dh_std::Buffer<GpsMsg> gps_pos_buf_;       // GPS位置の観測値
  dh_std::Buffer<VelMsg> gps_vel_buf_;       // GPS速度の観測値
  dh_kdl_msgs::PoseVel posevel_;             // 発行する状態
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数

  CartesianFilter cart_filter_;

  ros::Publisher posevel_pub_;
  ros::Subscriber filtered_imu_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_pos_sub_;
  ros::Subscriber gps_vel_sub_;

  void fillUnusedBuffers();
  void advertisePublishers();
  void registerSubscribers();
  bool allMsgReceived();
  bool initDataCollected();
  void initialize();
  void setZeroPositions();
  void updatePoseVelMsg();

  void filteredImuCb(const ImuMsg& imu);
  void barometerCb(const BarMsg& bar);
  void gpsPositionCb(const GpsMsg& gps);
  void gpsVelocityCb(const VelMsg& vel);
};
