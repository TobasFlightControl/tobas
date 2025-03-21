#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/transmission.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
TransmissionWidget::TransmissionWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  gear_ratio_ = new ParamGetterWidget_DoubleSpinBox("Gear Ratio", "");  // TODO
  gear_ratio_->setDecimals(2);
  gear_ratio_->setMinimum(1.);
  gear_ratio_->setValue(1.);
  rows->addWidget(gear_ratio_);

  rows->addStretch();
}

const char* TransmissionWidget::name() const
{
  return "Transmission";
}

bool TransmissionWidget::isValid()
{
  return true;
}

void TransmissionWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<TransmissionWidget>(src);

  gear_ratio_->setValue(derived->gear_ratio_->getValue());
}

YAML::Node TransmissionWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[gear_ratio_->name()] = gear_ratio_->getValue();

  return node;
}

void TransmissionWidget::load(const YAML::Node& node)
{
  gear_ratio_->setValue(node[gear_ratio_->name()].as<int>());
}

double TransmissionWidget::gearRatio() const
{
  return gear_ratio_->getValue();
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
