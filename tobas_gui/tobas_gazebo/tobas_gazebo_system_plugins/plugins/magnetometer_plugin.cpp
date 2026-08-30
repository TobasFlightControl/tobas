// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_gazebo_tools/math.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_geographic/geography.hpp>
#include <tobas_math/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_time_tools/util.hpp>

#include <tobas_msgs_adapter/magnetic_field.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/random.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
/**
 * @brief Gazebo Magnetometer plugin
 *
 * - Initial bias is assumed to be calibrated.
 */
class GazeboMagnetometerPlugin : public BaseNode,
                                 public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPostUpdate
{
public:
  explicit GazeboMagnetometerPlugin();

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
  int update_rate_;            // [Hz] Update rate
  gz::math::Vector3d offset_;  // [m] B_Pos_BS
  double lat_0_;               // [deg] Latitude north of the origin
  double lon_0_;               // [deg] Longitude east of the origin
  double alt_0_;               // [m] Altitude of the origin
  double noise_stddev_;        // [G]
  double hard_bias_norm_;      // [G]

  std::optional<RateManager> rate_manager_;

  const cmp::WorldPose* pose_W_;

  gz::math::Vector3d hard_bias_;  // [G]
  double lat_, lon_;              // [deg] Current position

  std::random_device rnd_dev_;
  std::optional<NormalDistribution3d> noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboMagnetometerPlugin::GazeboMagnetometerPlugin()
{
}

void GazeboMagnetometerPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_magnetometer_plugin", sdf);
  getSdfParams(sdf);

  rate_manager_.emplace(update_rate_);

  const auto sc = getWorldSphericalCoordinates(ecm);
  if (!sc) {
    TOBAS_EXIT(sc.error());
  }
  lat_0_ = sc->LatitudeReference().Degree();
  lon_0_ = sc->LongitudeReference().Degree();
  alt_0_ = sc->ElevationReference();

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);

  hard_bias_ = createUnitSpherePoint(rnd_dev_) * hard_bias_norm_;

  noise_.emplace(rnd_dev_, 0.0, noise_stddev_);

  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(topic::kMagneticField);
}

void GazeboMagnetometerPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Get the sensor pose.
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);

  // Compute latitude, longitude, and altitude from Cartesian coordinates.
  const auto coord = geography_.planeToGeodetic(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_);
  lat_ = coord.latitude;
  lon_ = coord.longitude;
  const auto alt = alt_0_ + W_Pos_WS.Z();

  // Compute the geomagnetic reference value from latitude, longitude, and altitude.
  // TODO: Consider WMM error.
  const auto mag = geography_.magneticField(lat_, lon_, alt, tim::yearFraction());

  // Compute geomagnetic field viewed from the body coordinate system.
  const gz::math::Vector3d field_W(mag.east, mag.north, mag.up);  // ENU coordinates
  const auto field_B = T_W_B.Rot().RotateVectorReverse(field_W);

  // Add noise and observe the value normalized by the geomagnetic scale.
  const auto field_meas = (field_B + noise_->get() + hard_bias_) / mag.total;  // [-]

  // Create message.
  auto mag_msg = std::make_unique<tobas_msgs::MagneticField>();
  ros2::timeChronoToMsg(info.simTime, mag_msg->header.stamp);
  mag_msg->header.frame_id = link_name_;
  vectorGazeboToKDL(field_meas, mag_msg->mag);

  // Publish message.
  mag_pub_->publish(std::move(mag_msg));
}

void GazeboMagnetometerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kNonNegative);
  getSdfParam(sdf, "offset", offset_);

  getSdfParam(sdf, "noiseStddev", noise_stddev_, kNonNegative);
  getSdfParam(sdf, "hardBiasNorm", hard_bias_norm_, kNonNegative);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboMagnetometerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
