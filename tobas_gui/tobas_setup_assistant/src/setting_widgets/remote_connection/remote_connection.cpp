#include "tobas_setup_assistant/setting_tabs/remote_connection/remote_connection.hpp"

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace sa
{
namespace rc
{
RemoteConnectionWidget::RemoteConnectionWidget()
{
  addWidget(new qt::Label(kHostLabel, common::kLabelPSize, QFont::Bold));
  addWidget(new qt::DescriptionWidget(
    "Specify the target FC host as either a hostname, an IPv4 address, or an IPv6 address.", common::kBodyPSize));

  host_ = new HostWidget();
  addWidget(host_);

  addStretch();
}

const char* RemoteConnectionWidget::name() const
{
  return "Remote Connection";
}

const char* RemoteConnectionWidget::title() const
{
  return "Specify SSH Endpoint";
}

const char* RemoteConnectionWidget::description() const
{
  return "Configure the settings required to connect remotely "
         "from the ground control station (GCS) to the flight controller (FC). "
         "Enter your flight controller’s settings in each field. "
         "These are GCS-side settings and do not change any configuration on the FC.";
}

void RemoteConnectionWidget::updateInternalDataStructures()
{
  return;
}

bool RemoteConnectionWidget::isValid()
{
  if (!host_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node RemoteConnectionWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kHostLabel] = host_->dump();

  return node;
}

void RemoteConnectionWidget::load(const YAML::Node& node)
{
  host_->load(node[kHostLabel]);
}

QString RemoteConnectionWidget::host() const
{
  return host_->host();
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
