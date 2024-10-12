#include <QButtonGroup>

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/speed_limit.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/manual.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/voltage.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/current.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
SpeedLimitWidget::SpeedLimitWidget(AerodynamicsWidget* aerodynamics, ElectrodynamicsWidget* electrodynamics)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addWidget(new qt::DescriptionWidget("", kBodyPSize));  // TODO

  methods_.push_back(new SpeedLimitWidget_Manual());
  methods_.push_back(new SpeedLimitWidget_Voltage(electrodynamics));
  methods_.push_back(new SpeedLimitWidget_Current(electrodynamics, aerodynamics));

  const auto ckb_group = new QButtonGroup(this);
  ckb_group->setExclusive(true);

  for (const auto& method : methods_)
  {
    method->initialize(ckb_group);
    rows->addWidget(method);
  }

  rows->addStretch();
}

const char* SpeedLimitWidget::name() const
{
  return "Speed Limit";
}

bool SpeedLimitWidget::isValid()
{
  for (const auto& method : methods_)
    if (method->isChecked())
      return method->isValid();

  qt::qErrorBox(this, "Please set maximum rotation speed.");
  return false;
}

void SpeedLimitWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qobject_cast<const SpeedLimitWidget*>(src);

  for (size_t i = 0; i < methods_.size(); ++i)
  {
    const auto& des_method = methods_.at(i);
    const auto& src_method = derived->methods_.at(i);
    des_method->copyFrom(src_method);
  }
}

YAML::Node SpeedLimitWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (const auto& method : methods_)
    node[method->name()] = method->dump();

  return node;
}

void SpeedLimitWidget::load(const YAML::Node& node)
{
  for (const auto& method : methods_)
    method->load(node[method->name()]);
}

double SpeedLimitWidget::maxRotSpeed() const
{
  return selected()->maxRotSpeed();
}

const SpeedLimitWidget_Base* SpeedLimitWidget::selected() const
{
  for (const auto& method : methods_)
    if (method->isChecked())
      return method;

  throw std::runtime_error("No method is selected.");
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
