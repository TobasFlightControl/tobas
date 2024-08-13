#include <tobas_math/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Gps.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
using namespace gz;
using namespace gz::math;
namespace cmp = sim::components;

namespace gazebo
{
/**
 * @brief GPSの位置データと速度データを発行するプラグイン．
 */
class GazeboGpsPlugin : public BaseNode, public sim::System, public sim::ISystemConfigure, public sim::ISystemPostUpdate
{
  // Default values
  static constexpr size_t kDefaultUpdateRate = 5;       // [Hz]
  static constexpr double kDefaultDelay = 0.1;          // [s]
  static constexpr double kDefaultPosCorrTime = 10.;    // [s]
  static constexpr double kDefaultHorPosAccuracy = 2;   // [m]
  static constexpr double kDefaultVerPosAccuracy = 4.;  // [m]
  static constexpr double kDefaultHorVelStdDev = 0.1;   // [m/s]
  static constexpr double kDefaultVerVelStdDev = 0.1;   // [m/s]

  using HistoryType = tuple<chrono::steady_clock::duration, Pose3d, Vector3d, Vector3d>;

public:
  explicit GazeboGpsPlugin();

  void Configure(
    const sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  string link_name_;
  Vector3d offset_;     // B_Pos_BS
  size_t update_rate_;  // 更新頻度 [Hz]
  double delay_;        // GPSの遅延時間 [s]
  double pos_corr_time_;
  double hor_pos_accuracy_;
  double ver_pos_accuracy_;
  double hor_vel_stddev_;
  double ver_vel_stddev_;
  double lat_0_;  // 原点の北緯
  double lon_0_;  // 原点の東経
  double alt_0_;  // 原点の高度

  RateManager::SharedPtr rate_manager_;

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;
  const cmp::AngularVelocity* gyro_B_;

  deque<HistoryType> history_;
  bool is_history_filled_;
  chrono::steady_clock::duration t_last_publish_;
  Vector3d pos_bias_ = Vector3d::Zero;

  random_device rnd_dev_;
  NormalDistribution3d::SharedPtr dpos_noise_;
  NormalDistribution3d::SharedPtr vel_noise_;

  // Publishers
  PublisherPtr<tobas_msgs::Gps> gps_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void setRandomDistribuitons();
  void fillCovariances(tobas_msgs::Gps& gps_msg);
  void updatePosition(tobas_msgs::Gps& gps_msg, const Pose3d& T_W_B);
  void updateVelocity(
    tobas_msgs::Gps& gps_msg,
    const Quaterniond& W_Rot_B,
    const Vector3d& W_Linvel_WB,
    const Vector3d& B_Angvel_WB);
};

GazeboGpsPlugin::GazeboGpsPlugin() : BaseNode("gps_plugin"), is_history_filled_(false)
{
}

void GazeboGpsPlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize(sdf);
  getSdfParams(sdf);
  setRandomDistribuitons();

  rate_manager_ = make_shared<RateManager>(update_rate_);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == sim::kNullEntity)
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  vel_W_ = getComponent<cmp::WorldLinearVelocity>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);

  gps_pub_ = createPublisher<tobas_msgs::Gps>(path::join(ns(), tobas::kGpsTopic));
}

void GazeboGpsPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);

  getSdfParam(sdf, "offset", offset_, Vector3d::Zero);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, NON_NEGATIVE);
  getSdfParam(sdf, "delay", delay_, kDefaultDelay, NON_NEGATIVE);
  getSdfParam(sdf, "positionCorrTime", pos_corr_time_, kDefaultPosCorrTime, POSITIVE);

  getSdfParam(sdf, "horPosAccuracy", hor_pos_accuracy_, kDefaultHorPosAccuracy, NON_NEGATIVE);
  getSdfParam(sdf, "verPosAccuracy", ver_pos_accuracy_, kDefaultVerPosAccuracy, NON_NEGATIVE);
  getSdfParam(sdf, "horVelStdDev", hor_vel_stddev_, kDefaultHorVelStdDev, NON_NEGATIVE);
  getSdfParam(sdf, "verVelStdDev", ver_vel_stddev_, kDefaultVerVelStdDev, NON_NEGATIVE);

  getSdfParam(sdf, "latitudeZero", lat_0_, kDefaultLatitudeZero);
  getSdfParam(sdf, "longitudeZero", lon_0_, kDefaultLongitudeZero);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero);
}

void GazeboGpsPlugin::setRandomDistribuitons()
{
  // 位置の乱数生成器
  // バイアスの絶対値の期待値が正確度に一致するように位置のSDEの標準偏差を定める (memo: 2-57)
  const auto hor_dpos_stddev = hor_pos_accuracy_ * sqrt(M_PI / pos_corr_time_);
  const auto ver_dpos_stddev = ver_pos_accuracy_ * sqrt(M_PI / pos_corr_time_);
  const Vector3d dpos_stddev(hor_dpos_stddev, hor_dpos_stddev, ver_dpos_stddev);
  dpos_noise_.reset(new NormalDistribution3d(rnd_dev_, Vector3d::Zero, dpos_stddev));

  // 速度の乱数生成器
  const Vector3d vel_stddev(hor_vel_stddev_, hor_vel_stddev_, ver_vel_stddev_);
  vel_noise_.reset(new NormalDistribution3d(rnd_dev_, Vector3d::Zero, vel_stddev));
}

void GazeboGpsPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager&)
{
  // 現在の状態を履歴に追加
  const auto& cur_time = info.simTime;
  history_.emplace_back(cur_time, pose_W_->Data(), vel_W_->Data(), gyro_B_->Data());

  // 古い履歴を削除
  while (chrono::duration<double>(cur_time - get<0>(history_.front())).count() > delay_)
  {
    history_.pop_front();
    if (!is_history_filled_)
      is_history_filled_ = true;
  }

  // オルンシュタイン＝ウーベンレック過程に従って位置のバイアスを更新
  const auto dt = chrono::duration<double>(info.dt).count();
  pos_bias_ += (dpos_noise_->get() - pos_bias_ / pos_corr_time_) * dt;

  // 更新時刻になっていなければ発行しない
  if (!rate_manager_->update(info.simTime))
    return;

  // 履歴が溜まっていなければ発行しない
  if (!is_history_filled_)
    return;

  // 最新の発行時刻を更新
  t_last_publish_ = cur_time;

  // 最も古い (= delay分遅れている) 状態を取得
  chrono::steady_clock::duration gps_time;
  Pose3d T_W_B;
  Vector3d W_Linvel_WB;
  Vector3d B_Angvel_WB;
  tie(gps_time, T_W_B, W_Linvel_WB, B_Angvel_WB) = history_.front();

  // GPSメッセージを作成
  auto gps_msg = make_unique<tobas_msgs::Gps>();
  gps_msg->header.frame_id = link_name_;
  ros2::timeChronoToMsg(gps_time, gps_msg->header.stamp);
  gps_msg->fix_type = tobas_msgs::msg::Gps::FIX_3D;
  fillCovariances(*gps_msg);
  updatePosition(*gps_msg, T_W_B);
  updateVelocity(*gps_msg, T_W_B.Rot(), W_Linvel_WB, B_Angvel_WB);

  // メッセージを発行
  gps_pub_->publish(move(gps_msg));
}

void GazeboGpsPlugin::fillCovariances(tobas_msgs::Gps& gps_msg)
{
  // FIXME: 正確度と共分散は異なる．しかしGNSSは白色ノイズではないし，どう共分散を計算している？
  gps_msg.position_covariance.setZero();
  gps_msg.position_covariance.diagonal()(0) = ::math::sqr(hor_pos_accuracy_);
  gps_msg.position_covariance.diagonal()(1) = ::math::sqr(hor_pos_accuracy_);
  gps_msg.position_covariance.diagonal()(2) = ::math::sqr(ver_pos_accuracy_);

  gps_msg.velocity_covariance.setZero();
  gps_msg.velocity_covariance.diagonal()(0) = ::math::sqr(hor_vel_stddev_);
  gps_msg.velocity_covariance.diagonal()(1) = ::math::sqr(hor_vel_stddev_);
  gps_msg.velocity_covariance.diagonal()(2) = ::math::sqr(ver_vel_stddev_);
}

void GazeboGpsPlugin::updatePosition(tobas_msgs::Gps& gps_msg, const Pose3d& T_W_B)
{
  // オフセットを考慮してGPSレシーバーの位置を計算
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);

  // Add the altitude of the origin to the z-coordinate
  W_Pos_WS.Z() += alt_0_;

  // 真値にバイアスを加える
  W_Pos_WS += pos_bias_;

  // Fill the GPS message
  tobas_std::cartToGpsRelative(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, gps_msg.latitude, gps_msg.longitude);
  gps_msg.altitude = W_Pos_WS.Z();
}

void GazeboGpsPlugin::updateVelocity(
  tobas_msgs::Gps& gps_msg,
  const Quaterniond& W_Rot_B,
  const Vector3d& W_Linvel_WB,
  const Vector3d& B_Angvel_WB)
{
  // オフセットを考慮してGPSレシーバの速度を計算
  auto W_Linvel_WS = W_Linvel_WB + W_Rot_B.RotateVector(B_Angvel_WB.Cross(offset_));

  // Apply noise to ground speed
  W_Linvel_WS += vel_noise_->get();

  // Fill the ground speed message.
  vectorGazeboToKDL(W_Linvel_WS, gps_msg.ground_speed);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboGpsPlugin,
  sim::System,
  gazebo::GazeboGpsPlugin::ISystemConfigure,
  gazebo::GazeboGpsPlugin::ISystemPostUpdate)
