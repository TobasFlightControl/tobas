#pragma once

#include <urdf/model.h>
#include <QObject>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_joint_axis_solver.hpp>
#include <tobas_drone_core/propulsion_system/rotor_axis.hpp>

namespace gui
{
namespace sa
{
class RobotInfo : public QObject
{
  Q_OBJECT

  static constexpr double kJntAxisCollinearTol = tobas_std::deg2rad(5);

Q_SIGNALS:
  void loaded();

public:
  explicit RobotInfo();

  bool loadFromPath(const std::string& path);

  const std::string& urdfText() const;
  urdf::ModelInterfaceConstSharedPtr urdf() const;
  const kdl::Tree& tree() const;

  const std::string& robotName() const;

  /* 指定したリンクの関節軸が，一般化座標に依らず指定した軸と平行であるかどうかを調べる． */
  bool isJntAxisAlwaysCollinear(const std::string& seg_name, const kdl::Vector& tar_axis);

  tobas::rotor_axis_t rotorAxisType(const std::string& seg_name);

private:
  // URDF information
  std::string urdf_text_;
  urdf::ModelInterfaceSharedPtr urdf_;
  kdl::Tree tree_;

  kdl::JntArray q_zeros_;
  kdl::TreeJointAxisSolver axis_solver_;
};
}  // namespace sa
}  // namespace gui
