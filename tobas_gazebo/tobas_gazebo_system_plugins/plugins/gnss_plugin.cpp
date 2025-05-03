#include <tobas_math/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/gnss.hpp>

#include <tobas_gazebo_tools/utils.hpp>

#include "../include/tobas_gazebo_system_plugins/common/common.hpp"
#include "../include/tobas_gazebo_system_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_system_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_system_plugins/random.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
/**
 * @brief GNSSの位置データと速度データを発行するプラグイン．
 */
class GazeboGnssPlugin : public BaseNode,
                         public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
  // Default values
  static constexpr size_t kDefaultUpdateRate = 5;       // [Hz]
  static constexpr double kDefaultDelay = 0.1;          // [s]
  static constexpr double kDefaultPosCorrTime = 10.;    // [s]
  static constexpr double kDefaultHorPosAccuracy = 2;   // [m]
  static constexpr double kDefaultVerPosAccuracy = 4.;  // [m]
  static constexpr double kDefaultHorVelStdDev = 0.1;   // [m/s]
  static constexpr double kDefaultVerVelStdDev = 0.1;   // [m/s]

  using HistoryType = tuple<chrono::steady_clock::duration, gz::math::Pose3d, gz::math::Vector3d, gz::math::Vector3d>;

public:
  explicit GazeboGnssPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  string link_name_;
  size_t update_rate_;         // 更新頻度 [Hz]
  gz::math::Vector3d offset_;  // B_Pos_BS
  double delay_;               // GNSSの遅延時間 [s]
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
  bool is_history_filled_ = false;
  chrono::steady_clock::duration t_last_publish_;
  gz::math::Vector3d pos_bias_ = gz::math::Vector3d::Zero;

  random_device rnd_dev_;
  NormalDistribution3d::SharedPtr dpos_noise_;
  NormalDistribution3d::SharedPtr vel_noise_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::Gnss> gnss_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void setRandomDistribuitons();
  void fillCovariances(tobas_msgs::Gnss& gnss_msg);
  void updatePosition(tobas_msgs::Gnss& gnss_msg, const gz::math::Pose3d& T_W_B);
  void updateVelocity(
    tobas_msgs::Gnss& gnss_msg,
    const gz::math::Quaterniond& W_Rot_B,
    const gz::math::Vector3d& W_Linvel_WB,
    const gz::math::Vector3d& B_Angvel_WB);
};

GazeboGnssPlugin::GazeboGnssPlugin()
{
}

void GazeboGnssPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_gnss_plugin", sdf);
  getSdfParams(sdf);
  setRandomDistribuitons();

  rate_manager_ = make_shared<RateManager>(update_rate_);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  vel_W_ = getComponent<cmp::WorldLinearVelocity>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);

  gnss_pub_ = createPublisher<tobas_msgs::Gnss>(tobas::kGnssTopic);
}

void GazeboGnssPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, NON_NEGATIVE);
  getSdfParam(sdf, "offset", offset_, gz::math::Vector3d::Zero);

  getSdfParam(sdf, "delay", delay_, kDefaultDelay, NON_NEGATIVE);
  getSdfParam(sdf, "positionCorrTime", pos_corr_time_, kDefaultPosCorrTime, POSITIVE);

  getSdfParam(sdf, "horPosAccuracy", hor_pos_accuracy_, kDefaultHorPosAccuracy, NON_NEGATIVE);
  getSdfParam(sdf, "verPosAccuracy", ver_pos_accuracy_, kDefaultVerPosAccuracy, NON_NEGATIVE);
  getSdfParam(sdf, "horVelStdDev", hor_vel_stddev_, kDefaultHorVelStdDev, NON_NEGATIVE);
  getSdfParam(sdf, "verVelStdDev", ver_vel_stddev_, kDefaultVerVelStdDev, NON_NEGATIVE);

  getSdfParam(sdf, "latitudeZero", lat_0_);
  getSdfParam(sdf, "longitudeZero", lon_0_);
  getSdfParam(sdf, "altitudeZero", alt_0_);
}

void GazeboGnssPlugin::setRandomDistribuitons()
{
  // 位置の乱数生成器
  // バイアスの絶対値の期待値が正確度に一致するように位置のSDEの標準偏差を定める (memo: 2-57)
  const auto hor_dpos_stddev = hor_pos_accuracy_ * sqrt(M_PI / pos_corr_time_);
  const auto ver_dpos_stddev = ver_pos_accuracy_ * sqrt(M_PI / pos_corr_time_);
  const gz::math::Vector3d dpos_stddev(hor_dpos_stddev, hor_dpos_stddev, ver_dpos_stddev);
  dpos_noise_.reset(new NormalDistribution3d(rnd_dev_, gz::math::Vector3d::Zero, dpos_stddev));

  // 速度の乱数生成器
  const gz::math::Vector3d vel_stddev(hor_vel_stddev_, hor_vel_stddev_, ver_vel_stddev_);
  vel_noise_.reset(new NormalDistribution3d(rnd_dev_, gz::math::Vector3d::Zero, vel_stddev));
}

void GazeboGnssPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  // 現在の状態を履歴に追加
  const auto& cur_time = info.simTime;
  history_.emplace_back(cur_time, pose_W_->Data(), vel_W_->Data(), gyro_B_->Data());

  // 古い履歴を削除
  while (chrono::duration<double>(cur_time - get<0>(history_.front())).count() > delay_) {
    history_.pop_front();
    if (!is_history_filled_) {
      is_history_filled_ = true;
    }
  }

  // オルンシュタイン＝ウーベンレック過程に従って位置のバイアスを更新
  const auto dt = chrono::duration<double>(info.dt).count();
  pos_bias_ += (dpos_noise_->get() - pos_bias_ / pos_corr_time_) * dt;

  // 更新時刻になっていなければ発行しない
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // 履歴が溜まっていなければ発行しない
  if (!is_history_filled_) {
    return;
  }

  // 最新の発行時刻を更新
  t_last_publish_ = cur_time;

  // 最も古い (= delay分遅れている) 状態を取得
  chrono::steady_clock::duration gnss_time;
  gz::math::Pose3d T_W_B;
  gz::math::Vector3d W_Linvel_WB;
  gz::math::Vector3d B_Angvel_WB;
  tie(gnss_time, T_W_B, W_Linvel_WB, B_Angvel_WB) = history_.front();

  // GNSSメッセージを作成
  auto gnss_msg = make_unique<tobas_msgs::Gnss>();
  gnss_msg->header.frame_id = link_name_;
  ros2::timeChronoToMsg(gnss_time, gnss_msg->header.stamp);
  gnss_msg->fix_type = tobas_msgs::msg::Gnss::FIX_3D;
  fillCovariances(*gnss_msg);
  updatePosition(*gnss_msg, T_W_B);
  updateVelocity(*gnss_msg, T_W_B.Rot(), W_Linvel_WB, B_Angvel_WB);

  // メッセージを発行
  gnss_pub_->publish(move(gnss_msg));
}

void GazeboGnssPlugin::fillCovariances(tobas_msgs::Gnss& gnss_msg)
{
  // FIXME: 正確度と共分散は異なる．しかしGNSSは白色ノイズではないし，どう共分散を計算している？
  gnss_msg.position_covariance.setZero();
  gnss_msg.position_covariance.diagonal()(0) = math::sqr(hor_pos_accuracy_);
  gnss_msg.position_covariance.diagonal()(1) = math::sqr(hor_pos_accuracy_);
  gnss_msg.position_covariance.diagonal()(2) = math::sqr(ver_pos_accuracy_);

  gnss_msg.velocity_covariance.setZero();
  gnss_msg.velocity_covariance.diagonal()(0) = math::sqr(hor_vel_stddev_);
  gnss_msg.velocity_covariance.diagonal()(1) = math::sqr(hor_vel_stddev_);
  gnss_msg.velocity_covariance.diagonal()(2) = math::sqr(ver_vel_stddev_);
}

void GazeboGnssPlugin::updatePosition(tobas_msgs::Gnss& gnss_msg, const gz::math::Pose3d& T_W_B)
{
  // オフセットを考慮してGNSSレシーバーの位置を計算
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);

  // Add the altitude of the origin to the z-coordinate
  W_Pos_WS.Z() += alt_0_;

  // 真値にバイアスを加える
  W_Pos_WS += pos_bias_;

  // Fill the GNSS message
  tobas_std::cartToGnssRelative(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, gnss_msg.latitude, gnss_msg.longitude);
  gnss_msg.altitude = W_Pos_WS.Z();
}

void GazeboGnssPlugin::updateVelocity(
  tobas_msgs::Gnss& gnss_msg,
  const gz::math::Quaterniond& W_Rot_B,
  const gz::math::Vector3d& W_Linvel_WB,
  const gz::math::Vector3d& B_Angvel_WB)
{
  // オフセットを考慮してGNSSレシーバの速度を計算
  auto W_Linvel_WS = W_Linvel_WB + W_Rot_B.RotateVector(B_Angvel_WB.Cross(offset_));

  // Apply noise to ground speed
  W_Linvel_WS += vel_noise_->get();

  // Fill the ground speed message.
  vectorGazeboToKDL(W_Linvel_WS, gnss_msg.ground_speed);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboGnssPlugin,
  gz::sim::System,
  gazebo::GazeboGnssPlugin::ISystemConfigure,
  gazebo::GazeboGnssPlugin::ISystemPostUpdate)
