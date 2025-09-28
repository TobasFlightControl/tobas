#include "tobas_setup_assistant/settings.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>

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
  remote_connection = new rc::RemoteConnectionWidget();
  controller = new ctrl::ControllerWidget();
  observer = new ObserverWidget();
  rc_input = new RcInputWidget();
  extra_joints = new ExtraJointsWidget(uadf, tree);
  failsafe = new FailsafeWidget();
  simulation = new SimulationWidget();
  author_info = new AuthorInformationWidget();

  // Basic settings
  basic_list_ = new qt::ListWidget();
  toolbox_->addItem(basic_list_, "Basic Settings");
  connect(basic_list_, &QListWidget::currentItemChanged, this, &self::onListItemChanged);
  addEntry(basic_list_, propulsion_system);
  addEntry(basic_list_, fixed_wing);
  addEntry(basic_list_, hardware);
  addEntry(basic_list_, remote_connection);

  // Additional settings
  additional_list_ = new qt::ListWidget();
  toolbox_->addItem(additional_list_, "Additional Settings");
  connect(additional_list_, &QListWidget::currentItemChanged, this, &self::onListItemChanged);
  addEntry(additional_list_, controller);
  addEntry(additional_list_, observer);
  addEntry(additional_list_, rc_input);
  addEntry(additional_list_, extra_joints);
  addEntry(additional_list_, failsafe);
  addEntry(additional_list_, simulation);
  addEntry(additional_list_, author_info);

  // Make mutually exclusive
  connect(basic_list_, &QListWidget::currentRowChanged, additional_list_, &qt::ListWidget::deselect);
  connect(additional_list_, &QListWidget::currentRowChanged, basic_list_, &qt::ListWidget::deselect);

  // Disable all pages
  for (int i = 0; i < stack_->count(); ++i) {
    setPageEnabled(i, false);
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
    setPageEnabled(i, true);
  }

  // 回転翼を持たない場合は設定を無効化
  if (uadf_.thrusts.size() == 0) {
    setPageEnabled(propulsion_system, false);
  }

  // 固定翼を持たない場合は設定を無効化
  if (uadf_.control_surfaces.size() == 0) {
    setPageEnabled(fixed_wing, false);
  }

  // 追加ジョイントを持たない場合は設定を無効化
  if (extra_joints->numJoints() == 0) {
    setPageEnabled(extra_joints, false);
  }

  // Default page
  setCurrentPage(0);
}

bool SettingsWidget::isValid()
{
  // 全ての設定項目について，単体で問題ないことを確認
  for (int i = 0; i < stack_->count(); ++i) {
    const auto cur_widget = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    if (!cur_widget->isValid()) {
      setCurrentPage(cur_widget);
      return false;
    }
  }

  switch (propulsion_system->type()) {
    case tobas::PropulsionSystem::kElectric: {
      // 電動モータのDShotチャンネルが設定されていることを確認
      for (const auto& elem : uadf_.thrusts) {
        const auto joint_name = QString::fromStdString(elem.first);
        if (!hardware->dshot()->contains(joint_name)) {
          qt::qWarnBox(this, "Please specify a DShot channel for electric rotor \"" + joint_name + "\".");
          setCurrentPage(hardware);
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
          qt::qWarnBox(this, "Please specify a PWM channel for variable pitch \"" + joint_name + "\".");
          setCurrentPage(hardware);
          return false;
        }
      }

      // エンジンスロットルのPWMチャンネルが設定されていることを確認
      if (!hardware->pwm()->contains(hw::PwmWidget::kEngineThrotLabel)) {
        qt::qWarnBox(this, "Please specify a PWM channel for engine throttle.");
        setCurrentPage(hardware);
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
      qt::qWarnBox(this, "Please specify a PWM channel for control surface \"" + joint_name + "\".");
      setCurrentPage(hardware);
      return false;
    }
  }

  // チルトジョイントのPWMチャンネルが設定されていることを確認
  for (const auto& elem : uadf_.tilts) {
    const auto joint_name = QString::fromStdString(elem.first);
    if (!hardware->pwm()->contains(joint_name)) {
      qt::qWarnBox(this, "Please specify a PWM channel for active tilt joint \"" + joint_name + "\".");
      setCurrentPage(hardware);
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

void SettingsWidget::setFrameType(FrameType type)
{
  // フレーム型が定義されていなければ制御器の設定を無効化
  if (type == FrameType::kUndefined) {
    setPageEnabled(controller, false);
  }

  controller->setFrameType(type);
}

int SettingsWidget::getIndex(BaseSettingWidget* page) const
{
  const auto idx = stack_->indexOf(page);
  TOBAS_CHECK(idx >= 0);
  return idx;
}

void SettingsWidget::addEntry(QListWidget* list, BaseSettingWidget* page)
{
  const auto idx = stack_->addWidget(page);
  const auto item = new QListWidgetItem(page->name(), list);
  item->setData(Qt::UserRole, idx);
}

void SettingsWidget::setCurrentPage(int idx)
{
  if (idx < basic_list_->count()) {
    toolbox_->setCurrentWidget(basic_list_);
    basic_list_->setCurrentRow(idx);
  }
  else {
    toolbox_->setCurrentWidget(additional_list_);
    additional_list_->setCurrentRow(idx - basic_list_->count());
  }
}

void SettingsWidget::setCurrentPage(BaseSettingWidget* page)
{
  setCurrentPage(getIndex(page));
}

void SettingsWidget::setPageEnabled(int idx, bool enabled)
{
  const auto page = stack_->widget(idx);
  page->setEnabled(enabled);

  if (idx < basic_list_->count()) {
    const auto item = basic_list_->item(idx);
    setListItemEnabled(item, enabled);
  }
  else {
    const auto item = additional_list_->item(idx - basic_list_->count());
    setListItemEnabled(item, enabled);
  }
}

void SettingsWidget::setPageEnabled(BaseSettingWidget* page, bool enabled)
{
  setPageEnabled(getIndex(page), enabled);
}

void SettingsWidget::setListItemEnabled(QListWidgetItem* item, bool enabled)
{
  constexpr auto kEnableFlags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;

  if (enabled) {
    item->setFlags(item->flags() | kEnableFlags);
  }
  else {
    item->setFlags(item->flags() & ~kEnableFlags);
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
