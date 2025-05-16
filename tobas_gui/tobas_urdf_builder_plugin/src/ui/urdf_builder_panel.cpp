#include "tobas_urdf_builder_plugin/ui/urdf_builder_panel.hpp"

#include <filesystem>

#include <rcutils/env.h>
#include <QMenu>
#include <QMessageBox>
#include <boost/polymorphic_cast.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include <tobas_constants/constants.hpp>

#include "ui_urdf_builder_panel.h"

#include "tobas_urdf_builder_plugin/ogre_helpers/static_link_updater.hpp"
#include "tobas_urdf_builder_plugin/ui/add_link_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/save_urdf_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/widget_item.hpp"
#include "tobas_urdf_builder_plugin/utils/constants.hpp"

#define ROBOT_MODEL_UPDATE_INTERVAL 10  // [ms]
#define INVALID_CHARS " '\"#$%&()^~|,.<>/\\!?"
#define TMP_URDF_PATH "/tmp/urdf_builder.urdf"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace urdf_builder
{
namespace ui
{
URDFBuilderPanel::URDFBuilderPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , node_manager_(0, nullptr, "urdf_builder")
  , node_(node_manager_.node())
  , property_client_(node_, tobas::kPropertyServerName, kPropertySection)
{
  setWindowTitle("URDF Builder");

  ui_ = new Ui::URDFBuilderPanelUI();
  ui_->setupUi(this);

  ui_->EnableVisualCheckBox->setChecked(kDefaultVisualVisible);
  ui_->EnableCollisionCheckBox->setChecked(kDefaultCollisionVisible);
  ui_->EnableInertiaCheckBox->setChecked(kDefaultInertiaVisible);

  link_dialog_ = new UpdateLinkDialog(node_, this);
  link_dialog_->hide();
  ui_->scrollAreaWidgetContents->layout()->addWidget(link_dialog_);

  defineConnections();
}

URDFBuilderPanel::~URDFBuilderPanel()
{
  delete link_dialog_;
}

void URDFBuilderPanel::onInitialize()
{
  Panel::onInitialize();

  ogre_ctrl_ = make_shared<ogre::OgreController>(getDisplayContext());
  update_timer_.start(ROBOT_MODEL_UPDATE_INTERVAL);
}

void URDFBuilderPanel::load(const rviz_common::Config& config)
{
  Panel::load(config);
}

void URDFBuilderPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

QStringList URDFBuilderPanel::linkNames() const
{
  return vm_.linkNames();
}

QStringList URDFBuilderPanel::jointNames() const
{
  return vm_.jointNames();
}

void URDFBuilderPanel::RobotNameTextChanged(const QString& name)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::RobotNameTextChanged");

  vm_.name(name.toStdString());
}

void URDFBuilderPanel::NewButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::NewButtonClicked");

  vm_.newRobot();

  ui_->Path->setText("");
  ui_->RobotName->clear();

  addRootLink();
  reload();
  selectRootLink();
}

void URDFBuilderPanel::LoadButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::LoadButtonClicked");

  // URDFまたはXACROのパスを取得
  const auto last_opened_dir = getLastOpenedDir();
  const auto file_path = QFileDialog::getOpenFileName(
    this, tr("Load URDF"), last_opened_dir, tr("Robot Description (*.urdf *.xacro);;All Files (*)"));

  if (file_path.isEmpty()) {
    return;
  }

  setLastOpenedDir(file_path);

  if (file_path.endsWith(".urdf")) {
    // URDFを解析
    if (!vm_.loadRobot(file_path)) {
      QMessageBox::warning(this, kError, "Failed to parse URDF.");
      return;
    }

    // URDFのパスを設定
    ui_->Path->setText(file_path);
  }
  else if (file_path.endsWith(".xacro")) {
    // XACROを展開
    const auto command = "xacro " + file_path + " > " + TMP_URDF_PATH;
    if (system(command.toUtf8()) != EXIT_SUCCESS) {
      QMessageBox::warning(this, kError, "Failed to convert XACRO to URDF.");
      return;
    }

    // URDFを解析
    if (!vm_.loadRobot(TMP_URDF_PATH)) {
      QMessageBox::warning(this, kError, "Failed to parse XACRO.");
      return;
    }

    // XACROをURDFで上書きするのはまずいため保存用パスを消去
    ui_->Path->clear();
  }
  else {
    QMessageBox::warning(this, kError, "Invalid file format: " + file_path);
    return;
  }

  ui_->RobotName->setText(QString::fromStdString(vm_.name()));

  reload();
  selectRootLink();
}

void URDFBuilderPanel::SaveButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::SaveButtonClicked");

  if (!isValid()) {
    return;
  }

  if (ui_->Path->text().isEmpty()) {
    SaveAsButtonClicked();
    return;
  }

  saveURDF(ui_->Path->text());
}

void URDFBuilderPanel::SaveAsButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::SaveAsButtonClicked");

  if (!isValid()) {
    return;
  }

  const auto last_opened_dir = getLastOpenedDir();
  SaveUrdfDialog dialog(this, last_opened_dir);

  const auto result = dialog.exec();
  if (result != QDialog::Accepted) {
    return;
  }
  const auto file_path = dialog.selectedFiles().first();
  assert(file_path.endsWith(".urdf"));

  setLastOpenedDir(file_path);

  if (!saveURDF(file_path)) {
    return;
  }

  ui_->Path->setText(file_path);
}

void URDFBuilderPanel::EnableVisualCheckBoxToggled(bool checked)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "URDFBuilderPanel::EnableVisualCheckBoxToggled(" << checked << ")");

  ogre_ctrl_->setVisualVisible(checked);
}

void URDFBuilderPanel::EnableCollisionCheckBoxToggled(bool checked)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "URDFBuilderPanel::EnableCollisiolCheckBoxToggled(" << checked << ")");

  ogre_ctrl_->setCollisionVisible(checked);
}

void URDFBuilderPanel::EnableInertiaCheckBoxToggled(bool checked)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "URDFBuilderPanel::EnableInertiaCheckBoxToggled(" << checked << ")");

  ogre_ctrl_->setInertiaVisible(checked);
}

void URDFBuilderPanel::LinkTreeWidgetItemClicked(QTreeWidgetItem* item, int)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::LinkTreeWidgetItemClicked");

  reflectSelectedItem(item);
}

void URDFBuilderPanel::LinkTreeWidgetItemChanged(QTreeWidgetItem* item, int)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::LinkTreeWidgetItemChanged");

  selectLink(item);

  const auto link_item = boost::polymorphic_downcast<LinkTreeWidgetItem*>(item);
  const auto link_name = link_item->viewModel()->name().toStdString();

  if (item->checkState(0) == Qt::Unchecked) {
    ogre_ctrl_->hide(link_name);
  }
  else {
    ogre_ctrl_->show(link_name);
  }
}

void URDFBuilderPanel::LinkTreeContextMenuRequested(const QPoint& point)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::LinkTreeContextMenuRequested");

  QMenu menu(this);
  menu.addAction(ui_->AddLinkAction);
  menu.addAction(ui_->CloneLinkAction);
  menu.addAction(ui_->RemoveLinkAction);
  menu.exec(ui_->LinkTreeWidget->mapToGlobal(point));
}

void URDFBuilderPanel::AddLinkActionToggled(bool)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::AddLinkActionToggled");

  // ルートリンクが存在する場合のみリンクの追加を許可
  if (!vm_.rootLinkViewModel()) {
    QMessageBox::warning(this, kError, "Please create a new robot model or load one first.");
    return;
  }

  const auto link_vm = make_shared<view_model::LinkViewModel>(nullptr);
  AddLinkDialog dialog(this, vm_.linkNames(), *link_vm);
  const auto result = dialog.exec();

  if (result != QDialog::Accepted) {
    return;
  }

  vm_.addLink(link_vm);
  reload();
}

void URDFBuilderPanel::RemoveLinkActionToggled(bool)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::RemoveLinkActionToggled");

  const auto& items = ui_->LinkTreeWidget->selectedItems();
  if (items.empty()) {
    QMessageBox::warning(this, kError, "No link is selected.");
    return;
  }

  const auto front = boost::polymorphic_downcast<LinkTreeWidgetItem*>(items.front());

  // ルートリンクは消せないようにする
  const auto& link = front->viewModel()->model();
  const auto& root_link = vm_.rootLinkViewModel()->model();
  if (link == root_link) {
    QMessageBox::warning(this, kError, "Root link cannot be removed.");
    return;
  }

  vm_.removeLink(front->viewModel());
  reload();
  link_dialog_->hide();
}

void URDFBuilderPanel::CloneLinkActionToggled(bool)
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::CloneLinkActionToggled");

  const auto& items = ui_->LinkTreeWidget->selectedItems();
  if (items.empty()) {
    QMessageBox::warning(this, kError, "No link is selected.");
    return;
  }

  const auto front = boost::polymorphic_downcast<LinkTreeWidgetItem*>(items.front());

  // ルートリンクは複製不可
  const auto& link = front->viewModel()->model();
  const auto& root_link = vm_.rootLinkViewModel()->model();
  if (link == root_link) {
    QMessageBox::warning(this, kError, "Root link cannot be cloned.");
    return;
  }

  vm_.cloneLink(front->viewModel());
  reload();
}

void URDFBuilderPanel::OnUpdate()
{
  ogre_ctrl_->update();
}

void URDFBuilderPanel::LinkDialogChanged()
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::LinkDialogChanged");

  vm_.updateLink(old_link_vm_, link_dialog_->viewModel());
  old_link_vm_ = link_dialog_->viewModel()->clone();  // 最後にURDFが更新されたときの設定を保持
  reload();
}

QString URDFBuilderPanel::getLastOpenedDir()
{
  string last_opened_dir;
  if (property_client_.get(kConfigKey_LastOpenedDir, last_opened_dir) < 0) {
    RCLCPP_WARN(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = rcutils_get_home_dir();
  }
  return QString::fromStdString(last_opened_dir);
}

void URDFBuilderPanel::setLastOpenedDir(const QString& file_path)
{
  fs::path p(file_path.toStdString());
  const auto dir = p.parent_path().string();

  if (property_client_.set(kConfigKey_LastOpenedDir, dir) < 0) {
    RCLCPP_WARN(node_->get_logger(), property_client_.errorMessage());
    return;
  }
  if (property_client_.save() < 0) {
    RCLCPP_WARN(node_->get_logger(), property_client_.errorMessage());
    return;
  }
}

void URDFBuilderPanel::defineConnections()
{
  connect(ui_->RobotName, &QLineEdit::textChanged, this, &self::RobotNameTextChanged);

  connect(ui_->LoadButton, &QPushButton::released, this, &self::LoadButtonClicked);
  connect(ui_->NewButton, &QPushButton::released, this, &self::NewButtonClicked);
  connect(ui_->SaveButton, &QPushButton::released, this, &self::SaveButtonClicked);
  connect(ui_->SaveAsButton, &QPushButton::released, this, &self::SaveAsButtonClicked);

  connect(ui_->EnableVisualCheckBox, &QCheckBox::toggled, this, &self::EnableVisualCheckBoxToggled);
  connect(ui_->EnableCollisionCheckBox, &QCheckBox::toggled, this, &self::EnableCollisionCheckBoxToggled);
  connect(ui_->EnableInertiaCheckBox, &QCheckBox::toggled, this, &self::EnableInertiaCheckBoxToggled);

  connect(ui_->LinkTreeWidget, &QTreeWidget::itemClicked, this, &self::LinkTreeWidgetItemClicked);
  connect(ui_->LinkTreeWidget, &QTreeWidget::itemChanged, this, &self::LinkTreeWidgetItemChanged);
  connect(ui_->LinkTreeWidget, &QTreeWidget::customContextMenuRequested, this, &self::LinkTreeContextMenuRequested);

  connect(ui_->AddLinkAction, &QAction::triggered, this, &self::AddLinkActionToggled);
  connect(ui_->RemoveLinkAction, &QAction::triggered, this, &self::RemoveLinkActionToggled);
  connect(ui_->CloneLinkAction, &QAction::triggered, this, &self::CloneLinkActionToggled);

  connect(&update_timer_, &QTimer::timeout, this, &self::OnUpdate);
  connect(link_dialog_, &UpdateLinkDialog::Changed, this, &self::LinkDialogChanged);
}

void URDFBuilderPanel::reload()
{
  RCLCPP_DEBUG(node_->get_logger(), "URDFBuilderPanel::reload");

  reloadLinkTree();
  reloadRobot();
}

void URDFBuilderPanel::reloadLinkTree()
{
  ui_->LinkTreeWidget->blockSignals(true);

  // 選択されているリンク名を取得
  QString selected_link_name = "";
  const auto& selected_items = ui_->LinkTreeWidget->selectedItems();
  if (!selected_items.empty()) {
    const auto front = boost::polymorphic_downcast<LinkTreeWidgetItem*>(selected_items.front());
    selected_link_name = front->viewModel()->name();
  }

  // チェック状態を取得
  QSet<QString> unchecked_links;
  for (int i = 0; i < ui_->LinkTreeWidget->topLevelItemCount(); ++i) {
    collectUncheckedLinks(ui_->LinkTreeWidget->topLevelItem(i), unchecked_links);
  }

  // 一度全てのノードをを削除
  ui_->LinkTreeWidget->clear();

  queue<pair<view_model::LinkViewModelPtr, QTreeWidgetItem*>> que;
  que.push({ vm_.rootLinkViewModel(), new LinkTreeWidgetItem(vm_.rootLinkViewModel(), ui_->LinkTreeWidget) });

  while (!que.empty()) {
    const auto t = que.front();
    que.pop();

    const auto& link_vm = t.first;
    const auto& item = t.second;

    item->setText(0, link_vm->name());
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);

    // 選択リンクが残っている場合は再び選択
    item->setSelected(link_vm->name() == selected_link_name);

    // チェック状態を保持
    if (unchecked_links.contains(link_vm->name())) {
      item->setCheckState(0, Qt::Unchecked);
    }
    else {
      item->setCheckState(0, Qt::Checked);
    }

    // 子ノードをキューに追加
    for (const auto& child : link_vm->children()) {
      const auto child_item = new LinkTreeWidgetItem(child);
      item->addChild(child_item);
      que.push({ child, child_item });
    }
  }

  ui_->LinkTreeWidget->expandAll();
  ui_->LinkTreeWidget->blockSignals(false);
}

void URDFBuilderPanel::reloadRobot()
{
  ogre_ctrl_->reload(vm_);
}

void URDFBuilderPanel::selectRootLink()
{
  const auto root_item = ui_->LinkTreeWidget->topLevelItem(0);
  selectLink(root_item);
}

void URDFBuilderPanel::selectLink(QTreeWidgetItem* item)
{
  ui_->LinkTreeWidget->clearSelection();
  item->setSelected(true);
  reflectSelectedItem(item);
}

void URDFBuilderPanel::reflectSelectedItem(QTreeWidgetItem* item)
{
  const auto link_item = boost::polymorphic_downcast<LinkTreeWidgetItem*>(item);
  const auto& link_vm = link_item->viewModel();
  const auto link_name = link_vm->name().toStdString();

  ogre_ctrl_->unhighlightAll();
  ogre_ctrl_->highlight(link_name);

  link_dialog_->show();
  link_dialog_->readFromVM(link_vm);  // リンクのビューモデルからダイアログの値を更新
  old_link_vm_ = link_vm->clone();    // リンクが選択された時点での設定を保持

  // ルートリンクだったら変更不可にする
  link_dialog_->setTabsEnabled(link_name != vm_.rootLink()->name);
}

void URDFBuilderPanel::addRootLink()
{
  const auto link_vm = make_shared<view_model::LinkViewModel>(nullptr);
  link_vm->name("root");
  vm_.addLink(link_vm);
}

bool URDFBuilderPanel::saveURDF(const QString& file_path)
{
  if (!vm_.saveRobot(file_path)) {
    QMessageBox::warning(this, kError, "Failed to save URDF.");
    return false;
  }

  return true;
}

bool URDFBuilderPanel::isValid()
{
  if (!vm_.rootLink()) {
    QMessageBox::warning(this, kError, "The robot is empty.");
    return false;
  }

  if (!isRobotNameValid()) {
    return false;
  }

  if (!isJointsValid()) {
    return false;
  }

  return true;
}

bool URDFBuilderPanel::isRobotNameValid()
{
  const auto name = ui_->RobotName->text();

  if (name.isEmpty()) {
    QMessageBox::warning(this, kError, "Please set robot name.");
    return false;
  }

  for (const auto& ch : INVALID_CHARS) {
    if (name.contains(ch)) {
      QMessageBox::warning(this, kError, "Robot name cannot contain '" + QString(ch) + "'.");
      return false;
    }
  }

  return true;
}

bool URDFBuilderPanel::isJointsValid()
{
  for (const auto& joint_pair : vm_.joints()) {
    const auto& name = joint_pair.first;
    const auto& joint = joint_pair.second;

    // 可動関節の軸が設定されていなければエラー
    const auto& type = joint->type;
    const auto& axis = joint->axis;

    if (
      type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC ||
      type == urdf::Joint::PLANAR) {
      if (axis.x == 0 && axis.y == 0 && axis.z == 0) {
        QMessageBox::warning(this, kError, "Please set the axis of the joint '" + QString::fromStdString(name) + "'.");
        return false;
      }
    }
  }

  return true;
}

void URDFBuilderPanel::collectUncheckedLinks(QTreeWidgetItem* item, QSet<QString>& set)
{
  if (item->checkState(0) == Qt::Unchecked) {
    const auto link_name = item->text(0);
    set.insert(link_name);
  }

  // 子アイテムを走査
  for (int i = 0; i < item->childCount(); ++i) {
    collectUncheckedLinks(item->child(i), set);
  }
}
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui

// rviz_common::Panelの派生クラスならばRvizのメインウィジェットにプラグインできる
PLUGINLIB_EXPORT_CLASS(gui::urdf_builder::ui::URDFBuilderPanel, rviz_common::Panel)
