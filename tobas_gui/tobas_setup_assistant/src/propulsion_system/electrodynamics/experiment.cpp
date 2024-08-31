#include <eigen3/Eigen/SVD>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/experiment.hpp"

namespace gui
{
namespace setup_assistant
{
ElectroDynamicsWidget_Experiment::ElectroDynamicsWidget_Experiment(rclcpp::Node::SharedPtr node) : node_(node)
{
}

const char* ElectroDynamicsWidget_Experiment::name() const
{
  return "Estimate from Motor Spec";
}

const char* ElectroDynamicsWidget_Experiment::description() const
{
  return "Estimate the motor dynamics from the motor's Kv value and internal registance.";
}

void ElectroDynamicsWidget_Experiment::onInit()
{
  data_ = new ParamGetterWidget_DoubleTable(node_, "Experimental data", { "Throttle", "Voltage", "RPM" }, "");
  data_->setDecimals({ 0, 6, 0 });
  data_->setMinimum({ 1.0, 1.0, 1.0 });
  data_->setMaximum({ 100., 1e+9, 1e+9 });
  data_->setSuffix({ " %", " V", " rpm" });
  data_->setFixedHeight(kTableHeight);
  data_->setColumnWidth(kTableColWidth);
  addWidget(data_);
}

bool ElectroDynamicsWidget_Experiment::isValid()
{
  if (data_->count() == 0)
  {
    qt::qErrorBox(this, "Measurements in static condition is blank.");
    return false;
  }

  return true;
}

void ElectroDynamicsWidget_Experiment::copyFrom(const ElectrodynamicsWidget_Base* src)
{
  const auto derived = qobject_cast<const ElectroDynamicsWidget_Experiment*>(src);
  data_->setValue(derived->data_->getValue());
}

YAML::Node ElectroDynamicsWidget_Experiment::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[data_->name()] = data_->getValue();

  return node;
}

void ElectroDynamicsWidget_Experiment::load(const YAML::Node& node)
{
  data_->setValue(node[data_->name()].as<Eigen::MatrixXd>());
}

std::pair<double, double> ElectroDynamicsWidget_Experiment::rotSpeedCoefs() const
{
  // TODO: 外れ値を除去
  // TODO: あまりにモデルからかけ離れていたら警告を出す

  // データを取得
  const auto data_mat = data_->getValue();
  const Eigen::VectorXd throttle = data_mat.col(0);                           // [%]
  const Eigen::VectorXd battery_voltage = data_mat.col(1);                    // [V]
  const Eigen::VectorXd rpm = data_mat.col(2);                                // [rpm]
  const Eigen::VectorXd motor_voltage = battery_voltage * (throttle / 100.);  // [V]
  const Eigen::VectorXd omega = rpm * tobas_std::kRpmToRps;                   // [rad/s]

  // 最小二乗法で係数を推定
  const Eigen::MatrixXd X = eigen_tools::concat(omega, omega.cwiseAbs2(), 1);
  const Eigen::Vector2d coefs = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(motor_voltage);
  return { coefs(0), coefs(1) };
}
}  // namespace setup_assistant
}  // namespace gui
