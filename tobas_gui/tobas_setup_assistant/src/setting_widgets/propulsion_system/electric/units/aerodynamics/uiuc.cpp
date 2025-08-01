#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/uiuc.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/constants.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/util.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget_UIUC::AerodynamicsWidget_UIUC(rclcpp::Node::SharedPtr node, const PropellerWidget* propeller)
  : propeller_(propeller)
{
  data_ = new ParamGetterWidget_DoubleTable(
    node,
    "Measurements in static condition",
    { "RPM", "CT", "CP" },
    "Please input experimental data from the Thrust Stand.");
  data_->setDecimals({ 3, 6, 6 });
  data_->setMinimum({ 1e-3, 1e-6, 1e-6 });
  data_->setSuffix({ " rpm", " N", " Nm" });
  data_->table()->setFixedHeight(kDataTableHeight);
  data_->table()->setColumnsWidth(kDataTableColWidth);
  addWidget(data_);
}

const char* AerodynamicsWidget_UIUC::name() const
{
  return "Estimate from UIUC Propeller Data";
}

const char* AerodynamicsWidget_UIUC::description() const
{
  return "If the propeller is listed in the "
         "<a href='https://m-selig.ae.illinois.edu/props/propDB.html'>UIUC Propeller Data Site</a>, "
         "aerodynamic constants measured by research institutions can be utilized.";
}

bool AerodynamicsWidget_UIUC::isValid()
{
  if (data_->count() == 0) {
    qt::qErrorBox(this, "Measurements in static condition is blank.");
    return false;
  }

  return true;
}

void AerodynamicsWidget_UIUC::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget_UIUC>(src);
  data_->setValue(derived->data_->getValue());
}

YAML::Node AerodynamicsWidget_UIUC::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[data_->name()] = data_->getValue();

  return node;
}

void AerodynamicsWidget_UIUC::load(const YAML::Node& node)
{
  data_->setValue(node[data_->name()].as<Eigen::MatrixXd>());
}

double AerodynamicsWidget_UIUC::motorConst() const
{
  const auto data_mat = data_->getValue();
  const auto ct = data_mat.col(1).eval();
  const auto d = propeller_->diameter();
  return motorConstFromUiuc(ct, d);
}

double AerodynamicsWidget_UIUC::momentConst() const
{
  const auto data_mat = data_->getValue();
  const auto ct = data_mat.col(1).eval();
  const auto cp = data_mat.col(2).eval();
  const auto d = propeller_->diameter();
  return momentConstFromUiuc(ct, cp, d);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
