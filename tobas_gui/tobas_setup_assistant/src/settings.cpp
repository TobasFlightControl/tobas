#include "tobas_setup_assistant/settings.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
SettingsWidget::SettingsWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf, const kdl::Tree& tree, Signals& sig)
  : uadf_(uadf)
{
  toolbox_ = new QToolBox();
  stack_ = new qt::StackedWidget();

  // Pages
  propulsion_system = new propulsion::PropulsionSystemWidget(node, uadf, sig);
  fixed_wing = new fw::FixedWingWidget(node, uadf);
  hardware = new hw::HardwareWidget(uadf, sig);
  controller = new ctrl::ControllerWidget();
  observer = new ObserverWidget();
  rc_input = new RcInputWidget();
  extra_joints = new ExtraJointsWidget(uadf, tree);
  pre_arm_check = new PreArmCheckWidget();
  simulation = new SimulationWidget();
  author_info = new AuthorInformationWidget();

  // Basic settings
  basic_list_ = new qt::ListWidget();
  toolbox_->addItem(basic_list_, "Basic Settings");
  connect(basic_list_, &QListWidget::currentItemChanged, this, &self::onListItemChanged);
  addEntry(basic_list_, propulsion_system);
  addEntry(basic_list_, fixed_wing);
  addEntry(basic_list_, hardware);

  // Additional settings
  additional_list_ = new qt::ListWidget();
  toolbox_->addItem(additional_list_, "Additional Settings");
  connect(additional_list_, &QListWidget::currentItemChanged, this, &self::onListItemChanged);
  addEntry(additional_list_, controller);
  addEntry(additional_list_, observer);
  addEntry(additional_list_, rc_input);
  addEntry(additional_list_, extra_joints);
  addEntry(additional_list_, pre_arm_check);
  addEntry(additional_list_, simulation);
  addEntry(additional_list_, author_info);

  // Make mutually exclusive
  connect(basic_list_, &QListWidget::currentRowChanged, additional_list_, &qt::ListWidget::deselect);
  connect(additional_list_, &QListWidget::currentRowChanged, basic_list_, &qt::ListWidget::deselect);

  // Default page
  basic_list_->setCurrentRow(0);

  // Disable all pages
  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    page->setEnabled(false);
  }

  // Layout
  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(toolbox_, 0);
  cols->addWidget(stack_, 1);
}

void SettingsWidget::updateInternalDataStructures()
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    page->updateInternalDataStructures();
    page->setEnabled(true);
  }
}

bool SettingsWidget::isValid()
{
  // 全ての設定項目について，単体で問題ないことを確認
  for (int i = 0; i < stack_->count(); ++i) {
    const auto cur_widget = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    if (!cur_widget->isValid()) {
      setCurrentWidget(cur_widget);
      return false;
    }
  }

  switch (propulsion_system->type()) {
    case tobas::PropulsionSystem::kElectric: {
      // 電動モータのDShotチャンネルが設定されていることを確認
      for (const auto& elem : uadf_.thrusts) {
        const auto joint_name = QString::fromStdString(elem.first);
        if (!hardware->dshot()->contains(joint_name)) {
          qt::qWarnBox(this, "Please specify basic_list_ DShot channel for electric rotor \"" + joint_name + "\".");
          setCurrentWidget(hardware);
          return false;
        }
      }

      break;
    }
    case tobas::PropulsionSystem::kIce: {
      // 可変ピッチプロペラのPWMチャンネルが設定されていることを確認
      for (const auto& elem : uadf_.thrusts) {
        const auto joint_name = QString::fromStdString(elem.first);
        if (!hardware->pwm()->contains(joint_name)) {
          qt::qWarnBox(this, "Please specify basic_list_ PWM channel for variable pitch \"" + joint_name + "\".");
          setCurrentWidget(hardware);
          return false;
        }
      }

      // エンジンスロットルのPWMチャンネルが設定されていることを確認
      if (!hardware->pwm()->contains(hw::PwmWidget::kEngineThrotLabel)) {
        qt::qWarnBox(this, "Please specify basic_list_ PWM channel for engine throttle.");
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
  for (const auto& elem : uadf_.control_surfaces) {
    const auto joint_name = QString::fromStdString(elem.first);
    if (!hardware->pwm()->contains(joint_name)) {
      qt::qWarnBox(this, "Please specify basic_list_ PWM channel for control surface \"" + joint_name + "\".");
      setCurrentWidget(hardware);
      return false;
    }
  }

  // ティルトジョイントのPWMチャンネルが設定されていることを確認
  for (const auto& elem : uadf_.tilts) {
    const auto joint_name = QString::fromStdString(elem.first);
    if (!hardware->pwm()->contains(joint_name)) {
      qt::qWarnBox(this, "Please specify basic_list_ PWM channel for active tilt joint \"" + joint_name + "\".");
      setCurrentWidget(hardware);
      return false;
    }
  }

  return true;
}

YAML::Node SettingsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qConstPointerCast<BaseSettingWidget>(stack_->widget(i));
    node[page->name()] = page->dump();
  }

  return node;
}

bool SettingsWidget::load(const YAML::Node& node)
{
  bool success = true;

  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    try {
      page->load(node[page->name()]);
    }
    catch (const std::exception& e) {
      qt::qErrorBox(this, "Failed to load settings of \"" + QString(page->name()) + "\":\n\n" + e.what());
      success = false;
    }
  }

  return success;
}

void SettingsWidget::addEntry(QListWidget* list, BaseSettingWidget* page)
{
  const auto idx = stack_->addWidget(page);
  const auto item = new QListWidgetItem(page->name(), list);
  item->setData(Qt::UserRole, idx);
}

void SettingsWidget::setCurrentWidget(BaseSettingWidget* page)
{
  const auto idx = stack_->indexOf(page);
  if (idx < 0) {
    qWarning() << "\"" << page->name() << "\" is not found.";
    return;
  }

  if (idx < basic_list_->count()) {
    toolbox_->setCurrentWidget(basic_list_);
    basic_list_->setCurrentRow(idx);
  }
  else {
    toolbox_->setCurrentWidget(additional_list_);
    additional_list_->setCurrentRow(idx - basic_list_->count());
  }
}

void SettingsWidget::onListItemChanged(QListWidgetItem* item)
{
  const auto idx = item->data(Qt::UserRole).toInt();
  if (idx < 0) {
    qWarning() << "Corresponding widget not found.";
    return;
  }
  stack_->setCurrentIndex(idx);
}
}  // namespace sa
}  // namespace gui
