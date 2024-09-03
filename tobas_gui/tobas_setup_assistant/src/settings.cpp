#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/settings.hpp"

namespace gui
{
namespace setup_assistant
{
SettingsWidget::SettingsWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot)
{
  battery = new BatteryWidget();
  propulsion_system = new propulsion_system::PropulsionSystemWidget(node, robot);
  fixed_wing = new fixed_wing::FixedWingWidget(node, robot);
  custom_joints = new CustomJointsWidget(robot);
  imu = new IMUWidget();
  magnetometer = new MagnetometerWidget();
  barometer = new BarometerWidget();
  gps = new GPSWidget();
  controller = new ControllerWidget(robot, propulsion_system, fixed_wing);
  observer = new ObserverWidget(robot, imu, barometer, gps);
  hardware = new HardwareWidget();
  simulation = new SimulationWidget();
  author_info = new AuthorInformationWidget();
  ros_package = new ROSPackageWidget(node, robot);

  addTab(battery, battery->name());
  addTab(propulsion_system, propulsion_system->name());
  addTab(fixed_wing, fixed_wing->name());
  addTab(custom_joints, custom_joints->name());
  addTab(imu, imu->name());
  addTab(magnetometer, magnetometer->name());
  addTab(barometer, barometer->name());
  addTab(gps, gps->name());
  addTab(controller, controller->name());
  addTab(observer, observer->name());
  addTab(hardware, hardware->name());
  addTab(simulation, simulation->name());
  addTab(author_info, author_info->name());
  addTab(ros_package, ros_package->name());

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

void SettingsWidget::onCurrentChanged(int index)
{
  const auto cur_widget = qobject_cast<BaseSettingWidget*>(widget(index));
  cur_widget->onOpened();
}
}  // namespace setup_assistant
}  // namespace gui
