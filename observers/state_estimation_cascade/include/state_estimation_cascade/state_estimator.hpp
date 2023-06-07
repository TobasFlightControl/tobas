#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>

#include <dh_std_tools/buffer.hpp>
#include <dh_ros_tools/node.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_msgs/LinearVelocityWithCovariance.h>
#include <tobas_msgs/BaseState.h>
#include <state_estimation_cascade/StateEstimationCascadeConfig.h>

#include "./cartesian_filter.hpp"

namespace state_estimation_cascade
{
class StateEstimator : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;
  using StateMsg = tobas_msgs::BaseState;

  using ConfigType = state_estimation_cascade::StateEstimationCascadeConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit StateEstimator();

private:
  bool is_initialized_;
  ros::Time t_last_;
  double lat_0_;                    // 緯度のゼロ点
  double lon_0_;                    // 経度のゼロ点
  double alt_0_;                    // 高度のゼロ点
  Eigen::Quaterniond quat_;         // 推定された姿勢
  Eigen::Vector2d xy_m_;            // 絶対平面位置の測定値 (world)
  Eigen::Vector3d v_m_;             // 絶対速度の測定値 (world)
  Eigen::Vector3d a_m_;             // 加速度の観測値 (local)
  dh_std::Buffer<ImuMsg> imu_buf_;  // 姿勢推定済みのIMUデータ
  dh_std::Buffer<BarMsg> bar_buf_;  // 気圧センサの観測値
  dh_std::Buffer<GpsMsg> gps_buf_;  // GPS位置の観測値
  dh_std::Buffer<VelMsg> vel_buf_;  // GPS速度の観測値
  StateMsg state_;                  // 発行する状態
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数

  CartesianFilter cart_filter_;

  // rosparams
  double gravity_;
  bool use_gps_;
  int imu_buf_size_;
  int bar_buf_size_;
  int gps_buf_size_;
  int vel_buf_size_;
  double grav_var_;

  // PubSub
  ros::Publisher posevel_pub_;
  ros::Subscriber filtered_imu_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_pos_sub_;
  ros::Subscriber gps_vel_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void setZeroPositions();
  void updatePoseVelMsg();

  void filteredImuCb(const ImuMsg& imu);
  void barometerCb(const BarMsg& bar);
  void gpsPositionCb(const GpsMsg& gps);
  void gpsVelocityCb(const VelMsg& vel);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
};
}  // namespace state_estimation_cascade
