#include <tobas_math/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>

#include <tobas_gazebo_tools/math.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboMagnetometerPlugin : public BaseNode,
                                 public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPostUpdate
{
  static constexpr size_t kDefaultUpdateRate = 100;  // [Hz]

public:
  explicit GazeboMagnetometerPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  string link_name_;
  size_t update_rate_;              // [Hz] Update rate
  gz::math::Vector3d offset_;       // [m] B_Pos_BS
  double lat_0_;                    // [deg] 原点の北緯
  double lon_0_;                    // [deg] 原点の東経
  double alt_0_;                    // [m] 原点の高度
  double noise_normal_;             // [nT]
  double noise_uniform_init_bias_;  // [nT]

  RateManager::SharedPtr rate_manager_;

  const cmp::WorldPose* pose_W_;

  gz::math::Vector3d init_bias_;  // [nT] 世界座標系の地磁気に加わるバイアス
  double lat_, lon_;              // [deg] 現在位置の経緯度

  random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticFieldStamped> mag_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboMagnetometerPlugin::GazeboMagnetometerPlugin() : rnd_gen_(rnd_dev_())
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

  rate_manager_ = make_shared<RateManager>(update_rate_);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity)
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);

  noise_ = NormalDistribution(0, noise_normal_);

  UniformDistribution init_bias_dist(-noise_uniform_init_bias_, noise_uniform_init_bias_);
  init_bias_.X(init_bias_dist(rnd_gen_));
  init_bias_.Y(init_bias_dist(rnd_gen_));
  init_bias_.Z(init_bias_dist(rnd_gen_));

  mag_pub_ = createPublisher<tobas_msgs::MagneticFieldStamped>(tobas::kMagRawTopic);
}

void GazeboMagnetometerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, NON_NEGATIVE);
  getSdfParam(sdf, "offset", offset_, gz::math::Vector3d::Zero);

  getSdfParam(sdf, "latitudeZero", lat_0_, kDefaultLatitudeZero);
  getSdfParam(sdf, "longitudeZero", lon_0_, kDefaultLongitudeZero);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero);

  getSdfParam(sdf, "noiseNormal", noise_normal_, 0., NON_NEGATIVE);
  getSdfParam(sdf, "noiseUniformInitialBias", noise_uniform_init_bias_, 0., NON_NEGATIVE);
}

void GazeboMagnetometerPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime))
    return;

  // Get the sensor pose
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);

  // デカルト座標から経緯度と高度を計算
  tobas_std::cartToGpsRelative(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, lat_, lon_);
  const auto alt = alt_0_ + W_Pos_WS.Z();

  // 経緯度と高度から地磁気の参照値を計算
  const auto mag = geomag::elementsFromGeodetic(lat_, lon_, alt, tobas_std::yearFraction());

  // 機体座標系から見た地磁気を計算
  const gz::math::Vector3d field_W(mag.north, -mag.east, -mag.down);  // [nT]
  auto field_B = T_W_B.Rot().RotateVectorReverse(field_W + init_bias_);

  // Add noise
  field_B.X() += noise_(rnd_gen_);
  field_B.Y() += noise_(rnd_gen_);
  field_B.Z() += noise_(rnd_gen_);

  // Create message
  auto mag_msg = make_unique<tobas_msgs::MagneticFieldStamped>();
  ros2::timeChronoToMsg(info.simTime, mag_msg->header.stamp);
  mag_msg->header.frame_id = link_name_;
  vectorGazeboToKDL(field_B.Normalized(), mag_msg->mag);

  // Publish message
  mag_pub_->publish(move(mag_msg));
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboMagnetometerPlugin,
  gz::sim::System,
  gazebo::GazeboMagnetometerPlugin::ISystemConfigure,
  gazebo::GazeboMagnetometerPlugin::ISystemPostUpdate)
