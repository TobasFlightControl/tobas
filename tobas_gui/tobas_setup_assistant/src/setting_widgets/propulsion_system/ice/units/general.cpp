#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/general.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
GeneralWidget::GeneralWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  direction_ = new ParamGetterWidget_ComboBox("Turning Direction", "");
  direction_->setChoices({ kCWName, kCCWName });
  rows->addWidget(direction_);

  rows->addStretch();
}

const char* GeneralWidget::name() const
{
  return "General";
}

bool GeneralWidget::isValid()
{
  return true;
}

void GeneralWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<GeneralWidget>(src);

  direction_->setValue(derived->direction_->getValue());
}

YAML::Node GeneralWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[direction_->name()] = direction_->getValue();

  return node;
}

void GeneralWidget::load(const YAML::Node& node)
{
  direction_->setValue(node[direction_->name()].as<QString>());
}

tobas::turning_direction_t GeneralWidget::direction() const
{
  const auto text = direction_->getValue();
  if (text == kCWName) {
    return tobas::turning_direction_t::CW;
  }
  else if (text == kCCWName) {
    return tobas::turning_direction_t::CCW;
  }
  else {
    throw;
  }
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
