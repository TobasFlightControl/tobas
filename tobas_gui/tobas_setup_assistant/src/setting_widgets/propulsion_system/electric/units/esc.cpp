#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/esc.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
ESCWidget::ESCWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  max_current_ = new ParamGetterWidget_SpinBox(
    "Maximum Current",
    "Maximum current that the ESC (Electronic Speed Controller) can safely handle. "
    "Exceeding this maximum current may lead to overheating or damage to the ESC, "
    "and in the worst case, it could cause failure or fire.");
  max_current_->setMinimum(1);
  max_current_->setValue(20);
  max_current_->setSuffix(" A");
  rows->addWidget(max_current_);

  rows->addStretch();
}

const char* ESCWidget::name() const
{
  return "ESC";
}

bool ESCWidget::isValid()
{
  return true;
}

void ESCWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<ESCWidget>(src);
  max_current_->setValue(derived->max_current_->getValue());
}

YAML::Node ESCWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[max_current_->name()] = max_current_->getValue();

  return node;
}

void ESCWidget::load(const YAML::Node& node)
{
  max_current_->setValue(node[max_current_->name()].as<int>());
}

double ESCWidget::maxCurrent() const
{
  return max_current_->getValue();
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
