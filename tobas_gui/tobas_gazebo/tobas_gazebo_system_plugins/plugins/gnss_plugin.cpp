// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <atomic>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_geographic/geography.hpp>
#include <tobas_math/core.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_msgs_adapter/gnss.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/random.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
/**
 * @brief Plugin that publishes GNSS position and velocity data.
 */
class GazeboGnssPlugin : public BaseNode,
                         public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
  using self = GazeboGnssPlugin;
  using HistoryType = std::tuple<ch::steady_clock::duration, gz::math::Pose3d, gz::math::Vector3d, gz::math::Vector3d>;
  using LoseFixSrv = std_srvs::srv::Trigger;

public:
  explicit GazeboGnssPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  geo::Geography geography_;

  // SDF parameters
  std::string link_name_;
  int update_rate_;            // Update rate [Hz]
  gz::math::Vector3d offset_;  // B_Pos_BS [m]
  double delay_;               // GNSS delay time [s]
  double pos_corr_time_;       // Correlation time constant of the OU process [s]
  double hor_pos_accuracy_;    // Horizontal position accuracy, expected error value [m]
  double ver_pos_accuracy_;    // Vertical position accuracy, expected error value [m]
  double hor_vel_stddev_;      // Standard deviation of horizontal velocity noise [m/s]
  double ver_vel_stddev_;      // Standard deviation of vertical velocity noise [m/s]
  double geoid_undulation_;    // WGS 84 ellipsoid height minus MSL height [m]

  double lat_0_;  // Latitude north of the origin [deg]
  double lon_0_;  // Longitude east of the origin [deg]
  double alt_0_;  // Altitude of the origin [m]

  RateManager::SharedPtr rate_manager_;

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;
  const cmp::AngularVelocity* gyro_B_;

  std::deque<HistoryType> history_;
  bool is_history_filled_ = false;
  ch::steady_clock::duration t_last_publish_;
  gz::math::Vector3d pos_bias_ = gz::math::Vector3d::Zero;
  std::atomic_bool force_no_fix_ = false;

  std::random_device rnd_dev_;
  NormalDistribution3d::SharedPtr dpos_noise_;
  NormalDistribution3d::SharedPtr vel_noise_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::Gnss> gnss_pub_;

  // Services
  ros2::ServiceServerPtr<LoseFixSrv> lose_fix_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void setRandomDistribuitons();
  void fillCovariances(tobas_msgs::Gnss& gnss_msg);
  void updatePosition(tobas_msgs::Gnss& gnss_msg, const gz::math::Pose3d& T_W_B);
  void updateVelocity(
    tobas_msgs::Gnss& gnss_msg,
    const gz::math::Quaterniond& W_Rot_B,
    const gz::math::Vector3d& W_Linvel_WB,
    const gz::math::Vector3d& B_Angvel_WB);

  void loseFixCb(const LoseFixSrv::Request::ConstSharedPtr& req, const LoseFixSrv::Response::SharedPtr& res);
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

  rate_manager_ = std::make_shared<RateManager>(update_rate_);

  const auto sc = getWorldSphericalCoordinates(ecm);
  if (!sc) {
    TOBAS_EXIT(sc.error());
  }
  lat_0_ = sc.value().LatitudeReference().Degree();
  lon_0_ = sc.value().LongitudeReference().Degree();
  alt_0_ = sc.value().ElevationReference();

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  vel_W_ = getComponent<cmp::WorldLinearVelocity>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);

  gnss_pub_ = createPublisher<tobas_msgs::Gnss>(topic::kGnss);
  lose_fix_ss_ = createService<LoseFixSrv>(kLoseGnssFixSrv, &self::loseFixCb, this);
}

void GazeboGnssPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  // Add the current state to history.
  const auto& cur_time = info.simTime;
  history_.emplace_back(cur_time, pose_W_->Data(), vel_W_->Data(), gyro_B_->Data());

  // Delete old history.
  while (ch::duration<double>(cur_time - std::get<0>(history_.front())).count() > delay_) {
    history_.pop_front();
    if (!is_history_filled_) {
      is_history_filled_ = true;
    }
  }

  // Update position bias according to an Ornstein-Uhlenbeck process.
  const auto dt = ch::duration<double>(info.dt).count();
  pos_bias_ += (dpos_noise_->get() - pos_bias_ / pos_corr_time_) * dt;

  // Do not publish unless it is time to update.
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Do not publish until enough history has accumulated.
  if (!is_history_filled_) {
    return;
  }

  // Update the latest publish time.
  t_last_publish_ = cur_time;

  // Get the oldest state, which is delayed by `delay`.
  const auto& [gnss_time, T_W_B, W_Linvel_WB, B_Angvel_WB] = history_.front();

  // Create the GNSS message.
  auto gnss_msg = std::make_unique<tobas_msgs::Gnss>();
  gnss_msg->header.frame_id = link_name_;
  ros2::timeChronoToMsg(gnss_time, gnss_msg->header.stamp);
  if (force_no_fix_.load()) {
    gnss_msg->fix_type = tobas_msgs::msg::Gnss::NO_FIX;
    gnss_msg->num_satellites_used = 0;
  }
  else {
    gnss_msg->fix_type = tobas_msgs::msg::Gnss::FIX_3D;
    gnss_msg->num_satellites_used = 20;  // TODO: Vary this appropriately.
  }
  fillCovariances(*gnss_msg);
  updatePosition(*gnss_msg, T_W_B);
  updateVelocity(*gnss_msg, T_W_B.Rot(), W_Linvel_WB, B_Angvel_WB);

  // Publish the message.
  gnss_pub_->publish(std::move(gnss_msg));
}

void GazeboGnssPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kNonNegative);
  getSdfParam(sdf, "offset", offset_, gz::math::Vector3d::Zero);

  getSdfParam(sdf, "delay", delay_, kNonNegative);
  getSdfParam(sdf, "positionCorrTime", pos_corr_time_, kPositive);

  getSdfParam(sdf, "horPosAccuracy", hor_pos_accuracy_, kNonNegative);
  getSdfParam(sdf, "verPosAccuracy", ver_pos_accuracy_, kNonNegative);
  getSdfParam(sdf, "horVelStdDev", hor_vel_stddev_, kNonNegative);
  getSdfParam(sdf, "verVelStdDev", ver_vel_stddev_, kNonNegative);

  getSdfParam(sdf, "geoidUndulation", geoid_undulation_, 0.0);
}

void GazeboGnssPlugin::setRandomDistribuitons()
{
  // Random generator for position.
  // Set the standard deviation of the position SDE so the expected absolute bias matches the accuracy (memo: 2-57).
  const auto hor_dpos_stddev = hor_pos_accuracy_ * std::sqrt(M_PI / pos_corr_time_);
  const auto ver_dpos_stddev = ver_pos_accuracy_ * std::sqrt(M_PI / pos_corr_time_);
  const gz::math::Vector3d dpos_stddev(hor_dpos_stddev, hor_dpos_stddev, ver_dpos_stddev);
  dpos_noise_.reset(new NormalDistribution3d(rnd_dev_, gz::math::Vector3d::Zero, dpos_stddev));

  // Random generator for velocity.
  const gz::math::Vector3d vel_stddev(hor_vel_stddev_, hor_vel_stddev_, ver_vel_stddev_);
  vel_noise_.reset(new NormalDistribution3d(rnd_dev_, gz::math::Vector3d::Zero, vel_stddev));
}

void GazeboGnssPlugin::fillCovariances(tobas_msgs::Gnss& gnss_msg)
{
  // FIXME: Accuracy and covariance are different. GNSS is not white noise, so how is covariance computed?
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
  // Compute GNSS receiver position while considering the offset.
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);

  // Add the altitude of the origin to the z-coordinate.
  W_Pos_WS.Z() += alt_0_;

  // Add bias to the true value.
  W_Pos_WS += pos_bias_;

  // Fill the GNSS message.
  const auto coord = geography_.planeToGeodetic(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_);
  gnss_msg.latitude = coord.latitude;
  gnss_msg.longitude = coord.longitude;
  gnss_msg.height_msl = W_Pos_WS.Z();
  gnss_msg.height_wgs84 = gnss_msg.height_msl + geoid_undulation_;
}

void GazeboGnssPlugin::updateVelocity(
  tobas_msgs::Gnss& gnss_msg,
  const gz::math::Quaterniond& W_Rot_B,
  const gz::math::Vector3d& W_Linvel_WB,
  const gz::math::Vector3d& B_Angvel_WB)
{
  // Compute GNSS receiver velocity while considering the offset.
  auto W_Linvel_WS = W_Linvel_WB + W_Rot_B.RotateVector(B_Angvel_WB.Cross(offset_));

  // Apply noise to ground speed.
  W_Linvel_WS += vel_noise_->get();

  // Fill the ground speed message.
  vectorGazeboToKDL(W_Linvel_WS, gnss_msg.ground_speed);
}

void GazeboGnssPlugin::loseFixCb(const LoseFixSrv::Request::ConstSharedPtr&, const LoseFixSrv::Response::SharedPtr& res)
{
  if (!force_no_fix_.exchange(true)) {
    res->message = "GNSS fix has been lost.";
  }
  else {
    res->message = "GNSS fix is already lost.";
  }

  res->success = true;
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboGnssPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
