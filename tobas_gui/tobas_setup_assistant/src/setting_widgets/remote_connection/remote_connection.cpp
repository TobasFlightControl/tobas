#include "tobas_setup_assistant/setting_tabs/remote_connection/remote_connection.hpp"

#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace sa
{
namespace rc
{
RemoteConnectionWidget::RemoteConnectionWidget()
{
  addWidget(new qt::Label(kHostLabel, kLabelPSize, QFont::Bold));

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
  return "Configure the settings required for remote communication with the flight controller. "
         "Please enter appropriate values in each field.";
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
}  // namespace rc
}  // namespace sa
}  // namespace gui
