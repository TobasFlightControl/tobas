#include "tobas_setup_assistant/settings.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
SettingsWidget::SettingsWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, Signals& sig) : robot_(robot)
{
  propulsion_system = new propulsion::PropulsionSystemWidget(node, robot, sig);
  fixed_wing = new fw::FixedWingWidget(node, robot);
  extra_joints = new ExtraJointsWidget(robot);
  rc_input = new RcInputWidget();
  controller = new ControllerWidget(robot);
  observer = new ObserverWidget();
  hardware = new hw::HardwareWidget(robot, sig);
  pre_arm_check = new PreArmCheckWidget();
  simulation = new SimulationWidget();
  author_info = new AuthorInformationWidget();

  // 各タブを追加
  addTab(propulsion_system, propulsion_system->name());
  // addTab(fixed_wing, fixed_wing->name());  // TODO
  addTab(extra_joints, extra_joints->name());
  addTab(rc_input, rc_input->name());
  addTab(controller, controller->name());
  addTab(observer, observer->name());
  addTab(hardware, hardware->name());
  addTab(pre_arm_check, pre_arm_check->name());
  addTab(simulation, simulation->name());
  addTab(author_info, author_info->name());

  // 各タブを初期化
  for (int i = 0; i < count(); ++i) {
    const auto tab = qt::qPointerCast<BaseSettingWidget>(widget(i));
    tab->setEnabled(false);  // 最初は無効化
  }

  // レイアウト
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  setTabSize(kTabWidth, kTabHeight);
}

void SettingsWidget::updateInternalDataStructures()
{
  for (int i = 0; i < count(); ++i) {
    const auto tab = qt::qPointerCast<BaseSettingWidget>(widget(i));
    tab->updateInternalDataStructures();
    tab->setEnabled(true);
  }
}

bool SettingsWidget::isValid()
{
  // 全ての設定項目について，単体で問題ないことを確認
  for (int i = 0; i < count(); ++i) {
    const auto cur_widget = qt::qPointerCast<BaseSettingWidget>(widget(i));
    if (!cur_widget->isValid()) {
      setCurrentWidget(cur_widget);
      return false;
    }
  }

  switch (propulsion_system->type()) {
    case tobas::propulsion_system_t::ELECTRIC: {
      // 電動モータのDShotチャンネルが設定されていることを確認
      for (const auto& elem : robot_.uadf().thrusts) {
        const auto joint_name = QString::fromStdString(elem.first);
        if (!hardware->dshot()->contains(joint_name)) {
          qt::qWarnBox(this, "Please specify a DShot channel for electric rotor \"" + joint_name + "\".");
          setCurrentWidget(hardware);
          return false;
        }
      }

      break;
    }
    case tobas::propulsion_system_t::ICE: {
      // 可変ピッチプロペラのPWMチャンネルが設定されていることを確認
      for (const auto& elem : robot_.uadf().thrusts) {
        const auto joint_name = QString::fromStdString(elem.first);
        if (!hardware->pwm()->contains(joint_name)) {
          qt::qWarnBox(this, "Please specify a PWM channel for variable pitch \"" + joint_name + "\".");
          setCurrentWidget(hardware);
          return false;
        }
      }

      // エンジンスロットルのPWMチャンネルが設定されていることを確認
      if (!hardware->pwm()->contains(hw::PwmWidget::kEngineThrotLabel)) {
        qt::qWarnBox(this, "Please specify a PWM channel for engine throttle.");
        setCurrentWidget(hardware);
        return false;
      }

      break;
    }
    default: {
      throw;
    }
  }

  // 固定翼の操舵面のPWMチャンネルが設定されていることを確認
  for (const auto& elem : robot_.uadf().control_surfaces) {
    const auto joint_name = QString::fromStdString(elem.first);
    if (!hardware->pwm()->contains(joint_name)) {
      qt::qWarnBox(this, "Please specify a PWM channel for control surface \"" + joint_name + "\".");
      setCurrentWidget(hardware);
      return false;
    }
  }

  // ティルトジョイントのPWMチャンネルが設定されていることを確認
  for (const auto& elem : robot_.uadf().tilts) {
    const auto joint_name = QString::fromStdString(elem.first);
    if (!hardware->pwm()->contains(joint_name)) {
      qt::qWarnBox(this, "Please specify a PWM channel for active tilt joint \"" + joint_name + "\".");
      setCurrentWidget(hardware);
      return false;
    }
  }

  // 観測不可能な情報を要求する制御コマンドが設定されている場合に警告
  // TODO

  return true;
}

YAML::Node SettingsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < count(); ++i) {
    const auto tab = qt::qConstPointerCast<BaseSettingWidget>(widget(i));
    node[tab->name()] = tab->dump();
  }

  return node;
}

bool SettingsWidget::load(const YAML::Node& node)
{
  bool success = true;

  for (int i = 0; i < count(); ++i) {
    const auto tab = qt::qPointerCast<BaseSettingWidget>(widget(i));
    try {
      tab->load(node[tab->name()]);
    }
    catch (const std::exception& e) {
      qt::qErrorBox(this, "Failed to load settings of \"" + QString(tab->name()) + "\":\n\n" + e.what());
      success = false;
    }
  }

  return success;
}
}  // namespace sa
}  // namespace gui
