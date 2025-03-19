#include <tobas_math/core.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/uiuc.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/blade_theory.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget_UIUC::AerodynamicsWidget_UIUC(rclcpp::Node::SharedPtr node, PropellerWidget* propeller)
  : node_(node), propeller_(propeller)
{
  data_ = new ParamGetterWidget_DoubleTable(
    node_, "Measurements in static condition", { "RPM", "CT", "CP" },
    "Please input experimental data from the Thrust Stand.");
  data_->setDecimals({ 3, 6, 6 });
  data_->setMinimum({ 1e-3, 1e-6, 1e-6 });
  data_->setSuffix({ " rpm", " N", " Nm" });
  data_->table()->setFixedHeight(kTableHeight);
  data_->table()->setColumnsWidth(kTableColWidth);
  addWidget(data_);
}

const char* AerodynamicsWidget_UIUC::name() const
{
  return "Estimate from UIUC Propeller Data (Recommended)";
}

const char* AerodynamicsWidget_UIUC::description() const
{
  return "If the propeller is listed in the "
         "<a href='https://m-selig.ae.illinois.edu/props/propDB.html'>UIUC Propeller Data Site</a>, "
         "aerodynamic constants measured by research institutions can be utilized.";
}

bool AerodynamicsWidget_UIUC::isValid()
{
  if (data_->count() == 0)
  {
    qt::qErrorBox(this, "Measurements in static condition is blank.");
    return false;
  }

  return true;
}

void AerodynamicsWidget_UIUC::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qobject_cast<const AerodynamicsWidget_UIUC*>(src);
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
  const auto CT = data_mat.col(1).mean();
  const auto d = propeller_->diameter();
  const auto rho = tobas_std::kStandardAirDensity;  // TODO: ランタイムの気圧変化を考慮
  return (CT * rho * math::quat(d)) / (4 * math::sqr(M_PI));
}

double AerodynamicsWidget_UIUC::momentConst() const
{
  const auto data_mat = data_->getValue();
  const auto CT = data_mat.col(1).mean();
  const auto CP = data_mat.col(2).mean();
  const auto d = propeller_->diameter();
  return (d * CP) / (2 * M_PI * CT);
}

double AerodynamicsWidget_UIUC::dragConst() const
{
  // TODO: ブレードの幾何形状のみから推定するのではなく，他の空力特性を考慮して推定
  const BladeTheory blade(
    propeller_->numBlade(), propeller_->radius(), propeller_->bladeChord(), propeller_->pitchAngle());
  return blade.dragConst();
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
