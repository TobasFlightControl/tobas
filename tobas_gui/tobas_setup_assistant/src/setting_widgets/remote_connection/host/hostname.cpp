#include "tobas_setup_assistant/setting_tabs/remote_connection/host/hostname.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace rc
{
HostnameWidget::HostnameWidget()
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  hostname_ = new QLineEdit();
  cols->addWidget(hostname_);
}

const char* HostnameWidget::label() const
{
  return "Hostname";
}

bool HostnameWidget::isValid()
{
  if (hostname_->text().isEmpty()) {
    qt::qWarnBox(this, "Please enter the flight controller’s hostname.");
    return false;
  }

  return true;
}

YAML::Node HostnameWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kHostnameKey] = hostname_->text();

  return node;
}

void HostnameWidget::load(const YAML::Node& node)
{
  const auto hostname = node[kHostnameKey].as<QString>();
  hostname_->setText(hostname);
}

QString HostnameWidget::host() const
{
  return hostname_->text() + ".local";
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
