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
  getRadio(0)->setChecked(true);
  updateEnabled();

  connect(group, &QButtonGroup::idClicked, this, &self::onButtonGroupIdClicked);
}

bool HostWidget::isValid()
{
  return getWidget(findCurrentRow())->isValid();
}

YAML::Node HostWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < rowCount(); ++i) {
    const auto widget = getWidget(i);
    node[widget->label()] = widget->dump();
  }

  return node;
}

void HostWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < rowCount(); ++i) {
    const auto widget = getWidget(i);
    widget->load(node[widget->label()]);
  }
}

QString HostWidget::host() const
{
  return getWidget(findCurrentRow())->host();
}

void HostWidget::addRow(qt::FormLayout* form, QButtonGroup* group, BaseHostWidget* widget)
{
  const auto button = new QRadioButton(widget->label());
  group->addButton(button, rowCount());
  form->addVAlignedRow(button, widget);

  Line line;
  line.button = button;
  line.widget = widget;
  lines_.append(line);
}

void HostWidget::updateEnabled()
{
  for (int i = 0; i < rowCount(); ++i) {
    getWidget(i)->setEnabled(getRadio(i)->isChecked());
  }
}

int HostWidget::rowCount() const
{
  return lines_.size();
}

int HostWidget::findCurrentRow() const
{
  for (int i = 0; i < rowCount(); ++i) {
    if (getRadio(i)->isChecked()) {
      return i;
    }
  }

  qWarning() << "No button is checked.";
  return -1;
}

QRadioButton* HostWidget::getRadio(int row)
{
  return lines_.at(row).button;
}

const QRadioButton* HostWidget::getRadio(int row) const
{
  return lines_.at(row).button;
}

BaseHostWidget* HostWidget::getWidget(int row)
{
  return lines_.at(row).widget;
}

const BaseHostWidget* HostWidget::getWidget(int row) const
{
  return lines_.at(row).widget;
}

void HostWidget::onButtonGroupIdClicked()
{
  updateEnabled();
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
