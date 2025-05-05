#include "tobas_setup_assistant/setting_tabs/observer/custom.hpp"

#include <QVBoxLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
CustomObserverWidget::CustomObserverWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  package_ = new ParamGetterWidget_LineEdit("Observer Package Name", "");
  rows->addWidget(package_);

  plugin_ = new ParamGetterWidget_LineEdit("Observer Plugin Name", "");
  rows->addWidget(plugin_);

  rows->addStretch();
}

const char* CustomObserverWidget::name() const
{
  return "Use Custom Observer";
}

const char* CustomObserverWidget::description() const
{
  return "";  // TODO: APIの案内など
}

QString CustomObserverWidget::observerPackage() const
{
  return package_->getValue();
}

QString CustomObserverWidget::pluginName() const
{
  return plugin_->getValue();
}

YAML::Node CustomObserverWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node CustomObserverWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[package_->name()] = package_->getValue();
  node[plugin_->name()] = plugin_->getValue();

  return node;
}

void CustomObserverWidget::load(const YAML::Node& node)
{
  package_->setValue(node[package_->name()].as<QString>());
  plugin_->setValue(node[plugin_->name()].as<QString>());
}

bool CustomObserverWidget::isValid()
{
  if (package_->getValue().isEmpty()) {
    qt::qErrorBox(this, "Please specify custom observer package name.");
    return false;
  }

  if (plugin_->getValue().isEmpty()) {
    qt::qErrorBox(this, "Please specify custom observer plugin name.");
    return false;
  }

  return true;
}
}  // namespace sa
}  // namespace gui
