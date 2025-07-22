#pragma once

#include <tinyxml2.h>
#include <QObject>
#include <QWidget>

#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_joint_axis_solver.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

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
  explicit RobotInfo(QWidget* parent);

  bool loadFromXml(const tinyxml2::XMLDocument* uadf_doc);
  bool loadFromText(const std::string& uadf_text);
  bool loadFromPath(const std::string& uadf_path);

  const uadf::Model& uadf() const;
  const kdl::Tree& tree() const;

  const std::string& robotName() const;
  const std::string& linkName(const std::string& joint_name) const;

  tinyxml2::XMLDocument* urdfDocument() const;
  std::string urdfText() const;

  /* 指定したリンクの関節軸が，一般化座標に依らず指定した軸と平行であるかどうかを調べる． */
  bool isJntAxisAlwaysCollinear(const std::string& link_name, const kdl::Vector& tar_axis);

private:
  QWidget* const parent_;

  // Model information
  uadf::Model uadf_;
  kdl::Tree tree_;

  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;

  kdl::JntArray q_zeros_;
  kdl::TreeJointParser jnt_parser_;
  kdl::TreeJointAxisSolver axis_solver_;

  bool loadCommon();
};
}  // namespace sa
}  // namespace gui
