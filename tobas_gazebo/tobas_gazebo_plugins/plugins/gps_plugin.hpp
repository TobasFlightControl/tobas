#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <tobas_msgs/Gps.h>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
/**
 * @brief GPSの位置データと速度データを発行するプラグイン．
 */
class GazeboGpsPlugin : public SensorPlugin
{
  // Constants
  static constexpr char kPluginName[] = "gps_plugin";

  // Default values
  static constexpr double kDefaultUpdateRate = 5.;      // [Hz]
  static constexpr double kDefaultDelay = 0.;           // [s]
  static constexpr double kDefaultPosCorrTime = 10.;    // [s]
  static constexpr double kDefaultHorPosAccuracy = 2;   // [m]
  static constexpr double kDefaultVerPosAccuracy = 4.;  // [m]
  static constexpr double kDefaultHorVelStdDev = 0.1;   // [m/s]
  static constexpr double kDefaultVerVelStdDev = 0.1;   // [m/s]

  using self = GazeboGpsPlugin;
  using super = SensorPlugin;
  using HistoryType =
    std::tuple<common::Time, ignition::math::Pose3d, ignition::math::Vector3d, ignition::math::Vector3d>;

public:
  explicit GazeboGpsPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  rclcpp::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  SdfVector3 offset_;   // B_Pos_BS
  double update_rate_;  // 更新頻度 [Hz]
  double delay_;        // GPSの遅延時間 [s]
  double pos_corr_time_;
  double hor_pos_accuracy_;
  double ver_pos_accuracy_;
  double hor_vel_stddev_;
  double ver_vel_stddev_;
  double lat_0_;  // 原点の北緯
  double lon_0_;  // 原点の東経
  double alt_0_;  // 原点の高度

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  std::deque<HistoryType> history_;
  bool is_history_filled_;
  common::Time t_last_loop_, t_last_publish_;
  ignition::math::Vector3d pos_bias_ = zero3;

  std::random_device rnd_dev_;
  NormalDistribution3dPtr dpos_noise_;
  NormalDistribution3dPtr vel_noise_;

  // Publishers
  rclcpp::Publisher gps_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void setRandomDistribuitons();
  void registerPublishers();
  void onUpdate();
  void fillCovariances(tobas_msgs::Gps& gps_msg);
  void updatePosition(tobas_msgs::Gps& gps_msg, const ignition::math::Pose3d& T_W_B);
  void updateVelocity(
    tobas_msgs::Gps& gps_msg,
    const ignition::math::Quaterniond& W_Rot_B,
    const ignition::math::Vector3d& W_Linvel_WB,
    const ignition::math::Vector3d& B_Angvel_WB);
};
}  // namespace gazebo
