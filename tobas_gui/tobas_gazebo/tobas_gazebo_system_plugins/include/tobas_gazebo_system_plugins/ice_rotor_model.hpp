#pragma once

#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>
#include <tobas_std_tools/range.hpp>

#include "./simple_joint_model.hpp"

namespace gazebo
{
class IceRotorModel;
using IceRotorModelMap = std::map<std::string, IceRotorModel>;

class IceRotorModel
{
public:
  bool initialize(const sdf::ElementConstPtr& sdf, gz::sim::EntityComponentManager& ecm, const gz::sim::Model& model);

  const std::string& getLinkName() const;
  int getDirection() const;
  double getGearRatio() const;
  size_t getNumBlades() const;

  double getMotorConst() const;
  double getMomentConst() const;
  double getDragConst() const;

  double getSpeed(const double& engine_speed) const;
  double getVelocity(const double& engine_speed) const;
  double getThrust(const double& engine_speed) const;
  double getTorque(const double& engine_speed) const;

  double getPitchAngle() const;

  void setTargetPitchAngle(const double& tar_pitch);

  void
  applyWrench(gz::sim::EntityComponentManager& ecm, const double& engine_speed, const gz::math::Vector3d& wind_vel_W);

  void updateJointPosition(gz::sim::EntityComponentManager& ecm, const double& engine_pos);

  void step(const double& dt);

private:
  // SDF parameters
  std::string link_name_;  // プロペラのリンク名
  int direction_;          // Turning direction: 1(CCW) or -1(CW)
  double gear_ratio_;      // 減速比 [-]
  size_t num_blades_;      // プロペラのブレード数
  tobas::VppMotorConstant motor_const_;
  tobas::VppMomentConstant moment_const_;
  tobas::VppDragConstant drag_const_;

  // Gazebo objects
  std::shared_ptr<gz::sim::Joint> joint_;
  std::shared_ptr<gz::sim::Link> link_;
  std::shared_ptr<gz::sim::Link> parent_link_;
  const gz::sim::components::JointAxis* jnt_axis_;
  const gz::sim::components::JointVelocity* jnt_vel_;
  const gz::sim::components::WorldPose* pose_W_;
  const gz::sim::components::WorldLinearVelocity* linvel_W_;
  const gz::sim::components::WorldAngularVelocity* angvel_W_;
  const gz::sim::components::Inertial* inertial_;

  // Other
  SimpleJointModel pitch_angle_;

  bool getSdfParams(const sdf::ElementConstPtr& sdf);
  bool initializeGazeboObjects(gz::sim::EntityComponentManager& ecm, const gz::sim::Model& model);
};
}  // namespace gazebo
