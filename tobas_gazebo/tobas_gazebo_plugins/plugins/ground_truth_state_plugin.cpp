#include <tobas_std_tools/geometry.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
class GazeboGroundTruthStatePlugin : public BaseNode,
                                     public sim::System,
                                     public sim::ISystemConfigure,
                                     public sim::ISystemPostUpdate
{
  static constexpr size_t kDefaultUpdateRate = 0;  // [Hz]

  using self = GazeboGroundTruthStatePlugin;

public:
  explicit GazeboGroundTruthStatePlugin();

  void Configure(
    const sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  size_t update_rate_;

  const cmp::WorldPose* pose_W_;
  const cmp::LinearVelocity* vel_B_;
  const cmp::AngularVelocity* gyro_B_;
  const cmp::LinearAcceleration* acc_B_;
  const cmp::AngularAcceleration* dgyro_B_;

  RateManager::SharedPtr rate_manager_;

  ros2::PublisherPtr<tobas_msgs::Odometry> odom_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboGroundTruthStatePlugin::GazeboGroundTruthStatePlugin()
{
}

void GazeboGroundTruthStatePlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize("gazebo_ground_truth_state_plugin", sdf);
  getSdfParams(sdf);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == sim::kNullEntity)
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  vel_B_ = getComponent<cmp::LinearVelocity>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);
  acc_B_ = getComponent<cmp::LinearAcceleration>(link, ecm);
  dgyro_B_ = getComponent<cmp::AngularAcceleration>(link, ecm);

  rate_manager_ = make_shared<RateManager>(update_rate_);

  odom_pub_ = createPublisher<tobas_msgs::Odometry>(path::join(ns(), kOdometryGtTopic));
}

void GazeboGroundTruthStatePlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, NON_NEGATIVE);
}

void GazeboGroundTruthStatePlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime))
    return;

  // Create Pose & Twist message
  auto odom = std::make_unique<tobas_msgs::Odometry>();
  odom->header.frame_id = link_name_;

  // Update time stamp
  ros2::timeChronoToMsg(info.simTime, odom->header.stamp);

  // Update status
  odom->status = tobas_msgs::msg::Odometry::NO_ERROR;

  // Update pose (Global)
  poseGazeboToKDL(pose_W_->Data(), odom->frame);

  // Update linear velocity (Local)
  vectorGazeboToKDL(vel_B_->Data(), odom->twist.vel);

  // Update angular velocity (Local)
  vectorGazeboToKDL(gyro_B_->Data(), odom->twist.rot);

  // Update linear acceleration (Local)
  vectorGazeboToKDL(acc_B_->Data(), odom->accel.linear);

  // Update angular acceleration (Local)
  vectorGazeboToKDL(dgyro_B_->Data(), odom->accel.angular);

  // Publish state message
  odom_pub_->publish(move(odom));
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboGroundTruthStatePlugin,
  sim::System,
  gazebo::GazeboGroundTruthStatePlugin::ISystemConfigure,
  gazebo::GazeboGroundTruthStatePlugin::ISystemPostUpdate)
