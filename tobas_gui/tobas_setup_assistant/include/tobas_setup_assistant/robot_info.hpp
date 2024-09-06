#pragma once

#include <QObject>

#include <hardware_interface/hardware_info.hpp>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/treejntaxissolver.hpp>
#include <tobas_drone_core/rotor_axis.hpp>

namespace gui
{
namespace setup_assistant
{
namespace hw_interface
{
enum type_t : int
{
  // Valid (>= 0)
  POSITION = 0,
  VELOCITY = 1,
  EFFORT = 2,

  // Invalid (< 0)
  NONE = -1,
  UNKNOWN = -2,
};

static constexpr char kPositionInterface[] = "hardware_interface/PositionJointInterface";
static constexpr char kVelocityInterface[] = "hardware_interface/VelocityJointInterface";
static constexpr char kEffortInterface[] = "hardware_interface/EffortJointInterface";
}  // namespace hw_interface

class RobotInfo : public QObject
{
  Q_OBJECT

  static constexpr double kJntAxisCollinearTol = tobas_std::deg2rad(5);

Q_SIGNALS:
  void loaded();

public:
  explicit RobotInfo();

  bool loadFromPath(const std::string& path);

  const kdl::Tree& tree() const;
  const hardware_interface::HardwareInfo& hardware() const;

  const std::string& robotName() const;

  hw_interface::type_t hardwareInterface(const std::string& jnt_name) const;

  /* 指定したリンクの関節軸が，一般化座標に依らず指定した軸と平行であるかどうかを調べる． */
  bool isJntAxisAlwaysCollinear(const std::string& seg_name, const kdl::Vector& tar_axis);

  tobas::rotor_axis_t rotorAxisType(const std::string& seg_name);

private:
  kdl::Tree tree_;
  hardware_interface::HardwareInfo hardware_;

  kdl::TreeJntAxisSolver axis_solver_;

  kdl::JntArray q_zeros_;
};
}  // namespace setup_assistant
}  // namespace gui
