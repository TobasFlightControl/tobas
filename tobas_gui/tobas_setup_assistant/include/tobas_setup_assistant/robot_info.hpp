#pragma once

#include <urdf/model.h>
#include <QObject>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_ros2_tools/simple_param_client.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/treejntaxissolver.hpp>
#include <tobas_drone_core/rotor_axis.hpp>

namespace gui
{
namespace setup_assistant
{
class RobotInfo : public QObject
{
  Q_OBJECT

  static constexpr double kJntAxisCollinearTol = tobas_std::deg2rad(5);

Q_SIGNALS:
  void loaded(const QString& urdf_content);

public:
  explicit RobotInfo(rclcpp::Node::SharedPtr node);

  bool loadFromPath(const std::string& path);

  urdf::ModelInterfaceConstSharedPtr urdf() const;
  const kdl::Tree& tree() const;

  const std::string& robotName() const;

  /* 指定したリンクの関節軸が，一般化座標に依らず指定した軸と平行であるかどうかを調べる． */
  bool isJntAxisAlwaysCollinear(const std::string& seg_name, const kdl::Vector& tar_axis);

  tobas::rotor_axis_t rotorAxisType(const std::string& seg_name);

private:
  rclcpp::Node::SharedPtr node_;
  ros2::SimpleParamClient rsp_client_;

  // URDF information
  urdf::ModelInterfaceSharedPtr urdf_;
  kdl::Tree tree_;

  kdl::TreeJntAxisSolver axis_solver_;
  kdl::JntArray q_zeros_;
};
}  // namespace setup_assistant
}  // namespace gui
