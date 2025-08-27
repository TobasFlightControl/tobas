#include "tobas_setup_assistant/setting_tabs/remote_connection/host/host.hpp"

#include <QDebug>

#include "tobas_setup_assistant/setting_tabs/remote_connection/host/hostname.hpp"
#include "tobas_setup_assistant/setting_tabs/remote_connection/host/ipv4.hpp"
#include "tobas_setup_assistant/setting_tabs/remote_connection/host/ipv6.hpp"

namespace gui
{
namespace sa
{
namespace rc
{
HostWidget::HostWidget()
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  const auto group = new QButtonGroup();
  group->setExclusive(true);

  addRow(form, group, new HostnameWidget());
  addRow(form, group, new IPv4Widget());
  addRow(form, group, new IPv6Widget());

  // Default
  buttons_.at(0)->setChecked(true);
  updateEnabled();

  connect(group, &QButtonGroup::idClicked, this, &self::onButtonGroupIdClicked);
}

bool HostWidget::isValid()
{
  return widgets_.at(findCurrentRow())->isValid();
}

YAML::Node HostWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < rowCount(); ++i) {
    const auto button = buttons_.at(i);
    const auto widget = widgets_.at(i);
    const auto label = widget->label();
    node[label][kIsCheckedKey] = button->isChecked();
    node[label][kSettingsKey] = widget->dump();
  }

  return node;
}

void HostWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < rowCount(); ++i) {
    const auto button = buttons_.at(i);
    const auto widget = widgets_.at(i);
    const auto sub_node = node[widget->label()];
    button->setChecked(sub_node[kIsCheckedKey].as<bool>());
    widget->load(sub_node[kSettingsKey]);
  }

  updateEnabled();
}

QString HostWidget::host() const
{
  return widgets_.at(findCurrentRow())->host();
}

void HostWidget::addRow(qt::FormLayout* form, QButtonGroup* group, BaseHostWidget* widget)
{
  const auto button = new QRadioButton(widget->label());
  group->addButton(button, rowCount());
  form->addVAlignedRow(button, widget);

  buttons_.append(button);
  widgets_.append(widget);
}

void HostWidget::updateEnabled()
{
  for (int i = 0; i < rowCount(); ++i) {
    widgets_.at(i)->setEnabled(buttons_.at(i)->isChecked());
  }
}

int HostWidget::rowCount() const
{
  assert(buttons_.size() == widgets_.size());
  return buttons_.size();
}

int HostWidget::findCurrentRow() const
{
  for (int i = 0; i < rowCount(); ++i) {
    if (buttons_.at(i)->isChecked()) {
      return i;
    }
  }

  qWarning() << "No button is checked.";
  return -1;
}

void HostWidget::onButtonGroupIdClicked()
{
  updateEnabled();
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
