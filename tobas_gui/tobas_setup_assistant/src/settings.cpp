#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/settings.hpp"

namespace gui
{
namespace sa
{
SettingsWidget::SettingsWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, Signals& _signals)
{
  propulsion_system = new propulsion::PropulsionSystemWidget(node, robot, _signals);
  fixed_wing = new fixed_wing::FixedWingWidget(node, robot);
  joint_config = new JointConfigurationWidget(robot, _signals, propulsion_system, fixed_wing);
  imu = new IMUWidget();
  magnetometer = new MagnetometerWidget();
  barometer = new BarometerWidget();
  gnss = new GNSSWidget();
  controller = new ControllerWidget(robot, propulsion_system, fixed_wing);
  observer = new ObserverWidget(robot, imu, barometer, gnss);
  hardware = new HardwareWidget();
  pre_arm_check = new PreArmCheckWidget();
  simulation = new SimulationWidget();
  author_info = new AuthorInformationWidget();
  ros_package = new ROSPackageWidget(node, robot);

  // 各タブを追加
  addTab(propulsion_system, propulsion_system->name());
  addTab(fixed_wing, fixed_wing->name());
  addTab(joint_config, joint_config->name());
  addTab(imu, imu->name());
  addTab(magnetometer, magnetometer->name());
  addTab(barometer, barometer->name());
  addTab(gnss, gnss->name());
  addTab(controller, controller->name());
  addTab(observer, observer->name());
  addTab(hardware, hardware->name());
  addTab(pre_arm_check, pre_arm_check->name());
  addTab(simulation, simulation->name());
  addTab(author_info, author_info->name());
  addTab(ros_package, ros_package->name());

  // 各タブを初期化
  for (int i = 0; i < count(); ++i)
  {
    const auto tab = qt::qPointerCast<BaseSettingWidget>(widget(i));
    tab->setEnabled(false);  // 最初は無効化
  }

  // レイアウト
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  setTabSize(kTabWidth, kTabHeight);

  // Connection
  connect(this, &self::currentChanged, this, &self::onCurrentChanged);
}

void SettingsWidget::updateInternalDataStructures()
{
  for (int i = 0; i < count(); ++i)
  {
    const auto tab = qt::qPointerCast<BaseSettingWidget>(widget(i));
    tab->updateInternalDataStructures();
    tab->setEnabled(true);
  }
}

bool SettingsWidget::isValid()
{
  // 全ての設定項目について，単体で問題ないことを確認
  for (int i = 0; i < count(); ++i)
  {
    const auto cur_widget = qt::qPointerCast<BaseSettingWidget>(widget(i));
    if (!cur_widget->isValid())
    {
      switchTab(cur_widget);
      return false;
    }
  }

  // PWMチャンネルが被ってないことを確認
  if (!isPwmChannelsUnique())
    return false;

  // 観測不可能な情報を要求する制御コマンドが設定されている場合に警告
  // TODO

  return true;
}

bool SettingsWidget::isPwmChannelsUnique()
{
  // 全てのPWMチャンネルを収集
  std::vector<int> channel_list;

  switch (propulsion_system->type())
  {
    case tobas::propulsion_system_t::ELECTRIC:
    {
      break;
    }
    case tobas::propulsion_system_t::ICE:
    {
      const auto iprop = qt::qConstPointerCast<propulsion::ice::PropulsionSystemWidget>(propulsion_system->selected());

      const auto engine = iprop->engine;
      channel_list.push_back(engine->hardwareIface()->pwmChannel());

      const auto units = iprop->units->selected();
      for (int i = 0; i < iprop->numUnits(); ++i)
      {
        const auto unit_widget = units->widget(i);
        const auto pwm_channel = unit_widget->hardwareIface()->pwmChannel();
        channel_list.push_back(pwm_channel);
      }

      break;
    }
    default:
    {
      throw;
    }
  }

  for (int i = 0; i < joint_config->numJoints(); ++i)
    if (joint_config->getHardwareInterface(i) == tobas::hw_iface_t::PWM)
      channel_list.push_back(joint_config->getPwmChannel(i));

  // PWMチャンネルがユニークであることを確認
  std::unordered_set<int> channel_set;
  for (const auto& channel : channel_list)
  {
    if (!channel_set.insert(channel).second)
    {
      qt::qErrorBox(this, "PWM channel " + QString::number(channel) + " is duplicated.");
      return false;
    }
  }

  return true;
}

YAML::Node SettingsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < count(); ++i)
  {
    const auto tab = qt::qConstPointerCast<BaseSettingWidget>(widget(i));
    node[tab->name()] = tab->dump();
  }

  return node;
}

bool SettingsWidget::load(const YAML::Node& node)
{
  bool success = true;

  for (int i = 0; i < count(); ++i)
  {
    const auto tab = qt::qPointerCast<BaseSettingWidget>(widget(i));
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
  const auto cur_widget = qt::qPointerCast<BaseSettingWidget>(widget(index));
  cur_widget->onOpened();
}
}  // namespace sa
}  // namespace gui
