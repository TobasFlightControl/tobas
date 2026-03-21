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
  host_ = new HostWidget();
  nif_ = new NetworkIfaceWidget();

  addWidget(new qt::Label(kNetworkIfaceLabel, cmn::kLabelPSize, QFont::Bold));
  addWidget(new qt::DescriptionWidget("Specify the network interface used by the flight controller.", cmn::kBodyPSize));
  addWidget(nif_);

  addWidget(new qt::Label(kHostLabel, cmn::kLabelPSize, QFont::Bold));
  addWidget(new qt::DescriptionWidget(
    "Specify the target FC host as either a hostname, an IPv4 address, or an IPv6 address.", cmn::kBodyPSize));
  addWidget(host_);

  addStretch();
}

const char* RemoteConnectionWidget::name() const
{
  return "Remote Connection";
}

const char* RemoteConnectionWidget::title() const
{
  return "Set up Remote Connection";
}

const char* RemoteConnectionWidget::description() const
{
  return "Configure the settings required to connect remotely "
         "from the ground control station (GCS) to the flight controller (FC). ";
}

void RemoteConnectionWidget::updateInternalDataStructures()
{
  return;
}

bool RemoteConnectionWidget::isValid()
{
  if (!nif_->isValid()) {
    return false;
  }
  if (!host_->isValid()) {
    return false;
  }

  return true;
}

YAML::Node RemoteConnectionWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNetworkIfaceLabel] = nif_->dump();
  node[kHostLabel] = host_->dump();

  return node;
}

void RemoteConnectionWidget::load(const YAML::Node& node)
{
  nif_->load(node[kNetworkIfaceLabel]);
  host_->load(node[kHostLabel]);
}

QString RemoteConnectionWidget::networkInterface() const
{
  return nif_->networkInterface();
}

QString RemoteConnectionWidget::host() const
{
  return host_->host();
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
