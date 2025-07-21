#include "tobas_setup_assistant/robot_info.hpp"

#include <urdf_parser/urdf_parser.h>

#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_xml_tools/core.hpp>

#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
RobotInfo::RobotInfo(QWidget* parent) : parent_(parent), jnt_parser_(tree_), axis_solver_(tree_)
{
}

bool RobotInfo::loadFromXml(const tinyxml2::XMLDocument* uadf_doc)
{
  if (!uadf_parser_.parseFromXml(uadf_doc, uadf_)) {
    qt::qErrorBox(parent_, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return false;
  }

  return loadCommon();
}

bool RobotInfo::loadFromText(const std::string& uadf_text)
{
  if (!uadf_parser_.parseFromText(uadf_text, uadf_)) {
    qt::qErrorBox(parent_, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return false;
  }

  return loadCommon();
}

bool RobotInfo::loadFromPath(const std::string& uadf_path)
{
  if (!uadf_parser_.parseFromPath(uadf_path, uadf_)) {
    qt::qErrorBox(parent_, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return false;
  }

  return loadCommon();
}

const uadf::Model& RobotInfo::uadf() const
{
  return uadf_;
}

const kdl::Tree& RobotInfo::tree() const
{
  return tree_;
}

const std::string& RobotInfo::robotName() const
{
  return uadf_.urdf->getName();
}

const std::string& RobotInfo::linkName(const std::string& joint_name) const
{
  return jnt_parser_.segmentName(joint_name);
}

tinyxml2::XMLDocument* RobotInfo::urdfDocument() const
{
// FIXME: Avoid deprecated function
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return urdf::exportURDF(*uadf_.urdf);
#pragma GCC diagnostic pop
}

std::string RobotInfo::urdfText() const
{
  const auto doc = urdfDocument();
  return xml::xmlDocumentToString(doc);
}

bool RobotInfo::isJntAxisAlwaysCollinear(const std::string& link_name, const kdl::Vector& tar_axis)
{
  const auto seg_it = tree_.getSegment(link_name);

  // 問題なくルートリンクまで遡れた場合はtrue．
  if (seg_it == tree_.getRootSegment()) {
    return true;
  }

  // ある関節角に対し，チェーンを構成する全てのジョイント軸が目標と平行であることが必要十分条件．
  // つまり，可動関節で且つジョイント軸が目標と平行でないリンクが存在する場合はfalse．
  const auto& joint = seg_it->second.segment.joint();
  if (joint.type != kdl::Joint::FIXED) {
    TOBAS_CHECK(axis_solver_.JntToCart(q_zeros_, link_name) == kdl::SolverI::E_NOERROR);
    const auto& cur_axis = axis_solver_.getAxis();
    if (cur_axis.argument(tar_axis) > kJntAxisCollinearTol) {
      return false;
    }
  }

  // 親リンクについて調べる
  const auto& par_name = seg_it->second.parent->first;
  return isJntAxisAlwaysCollinear(par_name, tar_axis);
}

tobas::rotor_axis_t RobotInfo::rotorAxisType(const std::string& link_name)
{
  if (isJntAxisAlwaysCollinear(link_name, kdl::Vector::UnitX())) {
    return tobas::rotor_axis_t::X_POSITIVE;
  }
  else if (isJntAxisAlwaysCollinear(link_name, kdl::Vector::UnitZ())) {
    return tobas::rotor_axis_t::Z_POSITIVE;
  }
  else {
    return tobas::rotor_axis_t::UNKNOWN;
  }
}

bool RobotInfo::loadCommon()
{
  // Check UADF validity
  if (!uadf_.valid()) {
    qt::qErrorBox(parent_, "UADF is invalid.");  // TODO: 詳細なエラーメッセージを表示
    return false;
  }

  // Load KDL tree
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      parent_, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    return false;
  }

  // Update KDL objects
  q_zeros_ = kdl::JntArray::Zero(tree_.getNrOfJoints());
  if (!jnt_parser_.updateInternalDataStructures()) {
    return false;
  }
  if (!axis_solver_.updateInternalDataStructures()) {
    return false;
  }

  Q_EMIT loaded();
  return true;
}
}  // namespace sa
}  // namespace gui
