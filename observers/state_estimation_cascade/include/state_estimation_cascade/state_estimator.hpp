#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Gps.h>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/StaticStateDeterminationAction.h>

#include <state_estimation_cascade/StateEstimationCascadeConfig.h>

#include "./cartesian_filter.hpp"

namespace state_estimation_cascade
{
class StateEstimator : public tobas::BaseNode
{
  using self = StateEstimator;
  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = tobas_msgs::Gps;
  using StateMsg = tobas_msgs::PoseTwist;
  using OdomMsg = nav_msgs::Odometry;

  using ConfigType = state_estimation_cascade::StateEstimationCascadeConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit StateEstimator(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // 固定値
  double lat_0_;  // 緯度のゼロ点
  double lon_0_;  // 経度のゼロ点
  double alt_0_;  // 高度のゼロ点

  bool is_initialized_ = false;
  bool imu_received_ = false;
  bool bar_received_ = false;
  bool gps_received_ = false;
  ros::Time t_last_;
  Eigen::Quaterniond quat_;  // 推定された姿勢
  Eigen::Vector2d xy_m_;     // 絶対平面位置の測定値 (world)
  Eigen::Vector3d a_m_;      // 加速度の観測値 (local)
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数

  CartesianFilter cart_filter_;

  // rosparams
  bool use_gps_;
  double gps_hor_pos_stddev_thr_;  // [m]
  double gps_ver_pos_stddev_thr_;  // [m]

  // PubSub
  ros::Publisher pt_pub_;
  ros::Publisher odom_pub_;
  ros::Subscriber filtered_imu_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize(const ImuMsg& imu);
  tobas_msgs::StaticStateDeterminationResultConstPtr setZeroPositions();
  StateMsg::ConstPtr makePoseVelMsg(const ImuMsg& imu);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void filteredImuCb(const ImuMsg::ConstPtr& imu);
  void barometerCb(const BarMsg::ConstPtr& bar);
  void gpsPositionCb(const GpsMsg::ConstPtr& gps);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace state_estimation_cascade
