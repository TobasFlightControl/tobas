#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_tools/math.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_math/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/time.hpp>

#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/conversions/conversions.hpp"
#include "tobas_gazebo_system_plugins/random.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
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
  // SDF parameters
  string link_name_;
  size_t update_rate_;         // [Hz] Update rate
  gz::math::Vector3d offset_;  // [m] B_Pos_BS
  double lat_0_;               // [deg] 原点の北緯
  double lon_0_;               // [deg] 原点の東経
  double alt_0_;               // [m] 原点の高度
  double noise_stddev_;        // [nT]
  double hard_bias_norm_;      // [nT]

  RateManager::SharedPtr rate_manager_;

  const cmp::WorldPose* pose_W_;

  gz::math::Vector3d hard_bias_;  // [nT]
  double lat_, lon_;              // [deg] Current position

  random_device rnd_dev_;
  NormalDistribution3d::SharedPtr noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticFieldStamped> mag_pub_;

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

  rate_manager_ = make_shared<RateManager>(update_rate_);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);

  hard_bias_ = createUnitSpherePoint(rnd_dev_) * hard_bias_norm_;

  noise_ = make_shared<NormalDistribution3d>(rnd_dev_, 0., noise_stddev_);

  mag_pub_ = createPublisher<tobas_msgs::MagneticFieldStamped>(tobas::kMagRawTopic);
}

void GazeboMagnetometerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, NON_NEGATIVE);
  getSdfParam(sdf, "offset", offset_);

  getSdfParam(sdf, "latitudeZero", lat_0_);
  getSdfParam(sdf, "longitudeZero", lon_0_);
  getSdfParam(sdf, "altitudeZero", alt_0_);

  getSdfParam(sdf, "noiseStddev", noise_stddev_, NON_NEGATIVE);
  getSdfParam(sdf, "hardBiasNorm", hard_bias_norm_, NON_NEGATIVE);
}

void GazeboMagnetometerPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Get the sensor pose
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B.RotateVector(offset_);

  // デカルト座標から経緯度と高度を計算
  tobas_std::cartToGnssRelative(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, lat_, lon_);
  const auto alt = alt_0_ + W_Pos_WS.Z();

  // 経緯度と高度から地磁気の参照値を計算
  // TODO: WMMの誤差を考慮
  const auto mag = geomag::elementsFromGeodetic(lat_, lon_, alt, tobas_std::yearFraction());

  // 機体座標系から見た地磁気を計算
  const gz::math::Vector3d field_W(mag.north, -mag.east, -mag.down);  // [nT]
  const auto field_B = T_W_B.Rot().RotateVectorReverse(field_W);      // [nT]

  // ノイズを加えて地磁気のスケールで正規化した値を観測する
  const auto field_meas = (field_B + noise_->get() + hard_bias_) / mag.total;  // [-]

  // Create message
  auto mag_msg = make_unique<tobas_msgs::MagneticFieldStamped>();
  ros2::timeChronoToMsg(info.simTime, mag_msg->header.stamp);
  mag_msg->header.frame_id = link_name_;
  vectorGazeboToKDL(field_meas, mag_msg->mag);

  // Publish message
  mag_pub_->publish(move(mag_msg));
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboMagnetometerPlugin,
  gz::sim::System,
  gazebo::GazeboMagnetometerPlugin::ISystemConfigure,
  gazebo::GazeboMagnetometerPlugin::ISystemPostUpdate)
