#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/transmission.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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

  gear_ratio_ = new ParamGetterWidget_Ratio("Gear Ratio", "");  // TODO
  gear_ratio_->setMinimum(1);
  gear_ratio_->setLeftText("Engine");
  gear_ratio_->setRightText("Propeller");
  gear_ratio_->setValue({ 60 * 33 * 23 * 39, 30 * 44 * 11 * 34 });
  rows->addWidget(gear_ratio_);

  rows->addStretch();
}

const char* TransmissionWidget::name() const
{
  return "Transmission";
}

bool TransmissionWidget::isValid()
{
  const auto [engine_gear, propeller_gear] = gear_ratio_->getValue();
  if (engine_gear < propeller_gear) {
    qt::qErrorBox(
      this, "The engine's rotational speed must be equal to or greater than the propeller's rotational speed.");
    return false;
  }

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
  gear_ratio_->setValue(node[gear_ratio_->name()].as<std::pair<int, int>>());
}

double TransmissionWidget::gearRatio() const
{
  const auto [engine_gear, propeller_gear] = gear_ratio_->getValue();
  return static_cast<double>(engine_gear) / static_cast<double>(propeller_gear);
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
