#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
class GazeboBarometerPlugin : public BaseNode,
                              public sim::System,
                              public sim::ISystemConfigure,
                              public sim::ISystemPostUpdate
{
  // Default values
  static constexpr size_t kDefaultUpdateRate = 50;   // [Hz]
  static constexpr double kDefaultPressureVar = 1.;  // [Pa]

  using PressureMsg = sensor_msgs::msg::FluidPressure;

public:
  explicit GazeboBarometerPlugin();

  void Configure(
    const sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  string link_name_;
  size_t update_rate_;
  math::Vector3d offset_;  // B_Pos_BS
  double alt_0_;
  double pressure_var_;

  const cmp::WorldPose* pose_W_;
  RateManager::SharedPtr rate_manager_;

  random_device rnd_dev_;
  mt19937 rnd_gen_;
  NormalDistribution pressure_noise_;

  ros2::PublisherPtr<sensor_msgs::msg::FluidPressure> pressure_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboBarometerPlugin::GazeboBarometerPlugin() : rnd_gen_(rnd_dev_())
{
}

void GazeboBarometerPlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize("gazebo_barometer_plugin", sdf);
  getSdfParams(sdf);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == sim::kNullEntity)
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  rate_manager_ = make_shared<RateManager>(update_rate_);
  pressure_noise_ = NormalDistribution(0., sqrt(pressure_var_));

  pressure_pub_ = createPublisher<PressureMsg>(path::join(ns(), tobas::kAirPressureTopic));
}

void GazeboBarometerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, math::Vector3d::Zero);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, NON_NEGATIVE);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero, NON_NEGATIVE);
  getSdfParam(sdf, "pressureVariance", pressure_var_, kDefaultPressureVar, NON_NEGATIVE);
}

void GazeboBarometerPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime))
    return;

  // Get the current geometric height of sensor
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);
  const auto altitude = alt_0_ + W_Pos_WS.Z();

  // Compute the air pressure at the current altitude
  auto pressure = tobas_std::altitudeToPressure(altitude);

  // Add noise to pressure measurement
  pressure += pressure_noise_(rnd_gen_);

  // Create a pressure message
  auto pressure_msg = make_unique<sensor_msgs::msg::FluidPressure>();
  ros2::timeChronoToMsg(info.simTime, pressure_msg->header.stamp);
  pressure_msg->header.frame_id = link_name_;
  pressure_msg->fluid_pressure = pressure;
  pressure_msg->variance = pressure_var_;

  // Publish the pressure message
  pressure_pub_->publish(move(pressure_msg));
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboBarometerPlugin,
  sim::System,
  gazebo::GazeboBarometerPlugin::ISystemConfigure,
  gazebo::GazeboBarometerPlugin::ISystemPostUpdate)
