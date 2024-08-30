#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/settings.hpp"

namespace gui
{
namespace setup_assistant
{
SettingsWidget::SettingsWidget()
{
  battery_ = new BatteryWidget();
  // TODO

  addTab(battery_, battery_->name());
  // TODO

  setMinimumHeight(kSettingsMinHeight);
  setStyleSheet(
    QString::fromStdString(std::format("QTabBar::tab {{ height: {}px; width: {}px; }}", kTabHeight, kTabWidth)));

  // 各タブを初期化
  for (int i = 0; i < count(); ++i)
  {
    auto tab = qobject_cast<BaseSettingWidget*>(widget(i));
    tab->initialize();
    tab->updateInternalDataStructures();
    tab->setEnabled(false);  // 最初は無効化
  }
}

void SettingsWidget::updateInternalDataStructures()
{
  for (int i = 0; i < count(); ++i)
  {
    auto tab = qobject_cast<BaseSettingWidget*>(widget(i));
    tab->updateInternalDataStructures();
    tab->setEnabled(true);
  }
}

YAML::Node SettingsWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < count(); ++i)
  {
    auto tab = qobject_cast<BaseSettingWidget*>(widget(i));
    node[tab->name()] = tab->dump();
  }

  return node;
}

bool SettingsWidget::load(const YAML::Node& node)
{
  bool success = true;

  for (int i = 0; i < count(); ++i)
  {
    auto tab = qobject_cast<BaseSettingWidget*>(widget(i));
    try
    {
      tab->onOpened();
      tab->load(node[tab->name()]);
    }
    catch (const std::exception& e)
    {
      qt::qErrorBox(this, "Failed to load settings of \"" + QString(tab->name()) + "\":\n\n" + e.what());
      success = false;
    }
  }

  return success;
}
}  // namespace setup_assistant
}  // namespace gui
