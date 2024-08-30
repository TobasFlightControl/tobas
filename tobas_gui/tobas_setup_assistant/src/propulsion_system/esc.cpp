#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_drone_core/esc.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/esc.hpp"

namespace gui
{
namespace setup_assistant
{
ESCWidget::ESCWidget()
  : signal_mode_map_{ { "BLHeli Open Loop", tobas::esc::kBLHeliOpenLoopName },
                      { "BHLeli Closed Loop (Low Range)", tobas::esc::kBLHeliCloseLoopLowName },
                      { "BHLeli Closed Loop (Middle Range)", tobas::esc::kBLHeliCloseLoopMidName },
                      { "BHLeli Closed Loop (High Range)", tobas::esc::kBLHeliCloseLoopHighName } }
{
  const auto rows = new QVBoxLayout(this);

  max_current_ = new ParamGetterWidget_SpinBox(
    "Maximum Current", "Maximum current that the ESC (Electronic Speed Controller) can safely handle. "
                       "Exceeding this maximum current may lead to overheating or damage to the ESC, "
                       "and in the worst case, it could cause failure or fire.");
  max_current_->setMinimum(1);
  max_current_->setValue(20);
  max_current_->setSuffix(" A");

  signal_mode_ = new ParamGetterWidget_ComboBox("Signal Mode", "");
  for (const auto& [text, _] : signal_mode_map_)
    signal_mode_->addChoice(text);
  rows->addWidget(signal_mode_);

  rows->addStretch();
}

const char* ESCWidget::name()
{
  return "ESC";
}

bool ESCWidget::isValid()
{
  return true;
}

void ESCWidget::copyFrom(const ESCWidget* src)
{
  max_current_->setValue(src->max_current_->getValue());
  signal_mode_->setValue(src->signal_mode_->getValue());
}

YAML::Node ESCWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[max_current_->name()] = max_current_->getValue();
  node[signal_mode_->name()] = signal_mode_->getValue();

  return node;
}

void ESCWidget::load(const YAML::Node& node)
{
  max_current_->setValue(node[max_current_->getValue()].as<int>());
  signal_mode_->setValue(node[signal_mode_->getValue()].as<QString>());
}

double ESCWidget::maxCurrent() const
{
  return max_current_->getValue();
}

std::string ESCWidget::signalMode() const
{
  return signal_mode_map_.at(signal_mode_->getValue());
}

}  // namespace setup_assistant
}  // namespace gui
