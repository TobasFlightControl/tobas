#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/thrust_stand.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/util.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget_ThrustStand::AerodynamicsWidget_ThrustStand(rclcpp::Node::SharedPtr node)
{
  data_ = new ParamGetterWidget_DoubleTable(
    node,
    "Data from thrust stand",
    { "RPM", "Thrust", "Torque" },
    "Please input experimental data from the Thrust Stand.");
  data_->setDecimals({ 0, 6, 6 });
  data_->setMinimum({ 1e-1, 1e-6, 1e-6 });
  data_->setSuffix({ " rpm", " N", " Nm" });
  data_->table()->setFixedHeight(kTableHeight);
  data_->table()->setColumnsWidth(kTableColWidth);
  addWidget(data_);
}

const char* AerodynamicsWidget_ThrustStand::name() const
{
  return "Estimate from Thrust Stand Data";
}

const char* AerodynamicsWidget_ThrustStand::description() const
{
  // XXX: テキスト中に改行コードを入れるとハイパーリンクが機能しない
  return "We estimate the aerodynamic constants from data obtained through Thrust Stand experiments. "
         "For example, see the "
         "<a href='https://www.tytorobotics.com/pages/series-1580-1585'>Tyto Rootics Series 1585 Thrust Stand</a>";
}

bool AerodynamicsWidget_ThrustStand::isValid()
{
  if (data_->count() == 0) {
    qt::qErrorBox(this, "Thrust stand data is blank.");
    return false;
  }

  return true;
}

void AerodynamicsWidget_ThrustStand::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget_ThrustStand>(src);
  data_->setValue(derived->data_->getValue());
}

YAML::Node AerodynamicsWidget_ThrustStand::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[data_->name()] = data_->getValue();

  return node;
}

void AerodynamicsWidget_ThrustStand::load(const YAML::Node& node)
{
  data_->setValue(node[data_->name()].as<Eigen::MatrixXd>());
}

double AerodynamicsWidget_ThrustStand::motorConst() const
{
  // TODO: 外れ値を除去
  // TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
  const auto data_mat = data_->getValue();
  const auto rpms = data_mat.col(0).eval();
  const auto thrusts = data_mat.col(1).eval();
  return motorConstFromThrustStand(rpms, thrusts);
}

double AerodynamicsWidget_ThrustStand::momentConst() const
{
  // TODO: 外れ値を除去
  // TODO: あまりにモデル(1次関数)からかけ離れていたら警告を出す
  const auto data_mat = data_->getValue();
  const auto thrusts = data_mat.col(1).eval();
  const auto torques = data_mat.col(2).eval();
  return momentConstFromThrustStand(thrusts, torques);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
