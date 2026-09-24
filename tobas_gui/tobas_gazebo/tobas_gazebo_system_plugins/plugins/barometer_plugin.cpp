// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>

#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>

#include <tobas_msgs/msg/fluid_pressure.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
class GazeboBarometerPlugin : public BaseNode,
                              public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPostUpdate
{
public:
  explicit GazeboBarometerPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  gz::math::Vector3d offset_;  // B_Pos_BS
  double alt_0_;               // [m]

  const cmp::WorldPose* pose_W_;
  std::optional<RateManager> rate_manager_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution pressure_noise_;

  ros2::PublisherPtr<tobas_msgs::msg::FluidPressure> pressure_pub_;
};

GazeboBarometerPlugin::GazeboBarometerPlugin() : rnd_gen_(rnd_dev_())
{
}

void GazeboBarometerPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_barometer_plugin", sdf);

  offset_ = getSdfParam<gz::math::Vector3d>(sdf, "offset", gz::math::Vector3d::Zero);

  const auto sc = getWorldSphericalCoordinates(ecm);
  if (!sc) {
    TOBAS_EXIT(sc.error());
  }
  alt_0_ = sc->ElevationReference();

  const gz::sim::Model model(model_entity);
  const auto link_name = getSdfParam<std::string>(sdf, "linkName");
  const auto link_enitty = model.LinkByName(ecm, link_name);
  if (link_enitty == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link '", link_name, "'.");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link_enitty, ecm);

  const auto update_rate = getSdfParam<int>(sdf, "updateRate", kNonNegative);
  rate_manager_.emplace(update_rate);

  const auto noise_stddev = getSdfParam<double>(sdf, "noiseStddev", kNonNegative);
  pressure_noise_ = NormalDistribution(0.0, noise_stddev);

  pressure_pub_ = createPublisher<tobas_msgs::msg::FluidPressure>(topic::kAirPressure);
}

void GazeboBarometerPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Get the current geometric height of sensor.
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);
  const auto altitude = alt_0_ + W_Pos_WS.Z();

  // Compute the air pressure at the current altitude.
  auto pressure = st::altitudeToPressure(altitude);

  // Add noise to pressure measurement.
  pressure += pressure_noise_(rnd_gen_);

  // Create a pressure message.
  auto pressure_msg = std::make_unique<tobas_msgs::msg::FluidPressure>();
  ros2::timeChronoToMsg(info.simTime, pressure_msg->header.stamp);
  pressure_msg->pressure = pressure;

  // Publish the pressure message.
  pressure_pub_->publish(std::move(pressure_msg));
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboBarometerPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
