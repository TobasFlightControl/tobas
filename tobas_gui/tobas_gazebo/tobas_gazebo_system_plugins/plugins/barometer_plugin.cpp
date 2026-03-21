#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>

#include <tobas_msgs/msg/fluid_pressure.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace cmp = gz::sim::components;

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
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  int update_rate_;
  gz::math::Vector3d offset_;  // B_Pos_BS
  double alt_0_;               // [m]
  double noise_stddev_;        // [Pa]

  const cmp::WorldPose* pose_W_;
  RateManager::SharedPtr rate_manager_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution pressure_noise_;

  ros2::PublisherPtr<tobas_msgs::msg::FluidPressure> pressure_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboBarometerPlugin::GazeboBarometerPlugin() : rnd_gen_(rnd_dev_())
{
}

void GazeboBarometerPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_barometer_plugin", sdf);
  getSdfParams(sdf);

  const auto sc = getWorldSphericalCoordinates(ecm);
  if (!sc) {
    TOBAS_EXIT(sc.error());
  }
  alt_0_ = sc.value().ElevationReference();

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  rate_manager_ = std::make_shared<RateManager>(update_rate_);
  pressure_noise_ = NormalDistribution(0., noise_stddev_);

  pressure_pub_ = createPublisher<tobas_msgs::msg::FluidPressure>(tobas::topic::kAirPressure);
}

void GazeboBarometerPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Get the current geometric height of sensor
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);
  const auto altitude = alt_0_ + W_Pos_WS.Z();

  // Compute the air pressure at the current altitude
  auto pressure = tbs::altitudeToPressure(altitude);

  // Add noise to pressure measurement
  pressure += pressure_noise_(rnd_gen_);

  // Create a pressure message
  auto pressure_msg = std::make_unique<tobas_msgs::msg::FluidPressure>();
  ros2::timeChronoToMsg(info.simTime, pressure_msg->header.stamp);
  pressure_msg->header.frame_id = link_name_;
  pressure_msg->pressure = pressure;

  // Publish the pressure message
  pressure_pub_->publish(std::move(pressure_msg));
}

void GazeboBarometerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, gz::math::Vector3d::Zero);
  getSdfParam(sdf, "updateRate", update_rate_, kNonNegative);
  getSdfParam(sdf, "noiseStddev", noise_stddev_, kNonNegative);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboBarometerPlugin,
  gz::sim::System,
  gazebo::GazeboBarometerPlugin::ISystemConfigure,
  gazebo::GazeboBarometerPlugin::ISystemPostUpdate)
