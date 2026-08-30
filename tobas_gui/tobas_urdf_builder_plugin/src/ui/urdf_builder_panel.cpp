// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ui/urdf_builder_panel.hpp"

#include <QDebug>
#include <QMenu>
#include <QMessageBox>
#include <boost/polymorphic_cast.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include "ui_urdf_builder_panel.h"

#include "tobas_urdf_builder_plugin/ogre_helpers/static_link_updater.hpp"
#include "tobas_urdf_builder_plugin/ui/add_link_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/save_urdf_dialog.hpp"
#include "tobas_urdf_builder_plugin/ui/update_link_dialog.hpp"
#include "tobas_urdf_builder_plugin/utils/widget_item.hpp"

#define ROBOT_MODEL_UPDATE_INTERVAL 10  // [ms]
#define INVALID_CHARS " '\"#$%&()^~|,.<>/\\!?"
#define TMP_URDF_PATH "/tmp/urdf_builder.urdf"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
UrdfBuilderPanel::UrdfBuilderPanel(QWidget* parent)
  : rviz_common::Panel(parent), settings_store_("tobas", "urdf_builder")
{
  ui_ = new Ui::URDFBuilderPanelUI();
  ui_->setupUi(this);

  ui_->EnableVisualCheckBox->setChecked(kDefaultVisualVisible);
  ui_->EnableCollisionCheckBox->setChecked(kDefaultCollisionVisible);
  ui_->EnableInertiaCheckBox->setChecked(kDefaultInertiaVisible);

  link_dialog_ = new UpdateLinkDialog(this);
  link_dialog_->hide();
  ui_->scrollAreaWidgetContents->layout()->addWidget(link_dialog_);

  defineConnections();
}

UrdfBuilderPanel::~UrdfBuilderPanel()
{
  delete link_dialog_;
}

void UrdfBuilderPanel::onInitialize()
{
  Panel::onInitialize();

  ogre_ctrl_.emplace(getDisplayContext());
  update_timer_.start(ROBOT_MODEL_UPDATE_INTERVAL);
}

void UrdfBuilderPanel::load(const rviz_common::Config& config)
{
  Panel::load(config);
}

void UrdfBuilderPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

QStringList UrdfBuilderPanel::linkNames() const
{
  return vm_.linkNames();
}

QStringList UrdfBuilderPanel::jointNames() const
{
  return vm_.jointNames();
}

void UrdfBuilderPanel::onRobotNameTextChanged(const QString& name)
{
  qDebug() << "UrdfBuilderPanel::onRobotNameTextChanged";

  vm_.name(name.toStdString());
}

void UrdfBuilderPanel::onNewButtonClicked()
{
  qDebug() << "UrdfBuilderPanel::onNewButtonClicked";

  vm_.newRobot();

  ui_->Path->setText("");
  ui_->RobotName->clear();

  addRootLink();
  reload();
  selectRootLink();
}

void UrdfBuilderPanel::onLoadButtonClicked()
{
  qDebug() << "UrdfBuilderPanel::onLoadButtonClicked";

  // Get the URDF or XACRO path.
  const auto last_opened_dir = getLastOpenedDir();
  const auto file_path = QFileDialog::getOpenFileName(
    this,
    "Load URDF",
    last_opened_dir,
    "Robot Description (*.urdf *.xacro);;All Files (*)",
    nullptr,
    QFileDialog::DontUseNativeDialog);

  if (file_path.isEmpty()) {
    return;
  }

  setLastOpenedDir(file_path);

  if (file_path.endsWith(".urdf")) {
    // Parse the URDF.
    if (!vm_.loadRobot(file_path)) {
      QMessageBox::warning(this, kError, "Failed to parse URDF.");
      return;
    }

    // Set the URDF path.
    ui_->Path->setText(file_path);
  }
  else if (file_path.endsWith(".xacro")) {
    // Expand XACRO.
    const auto command = "xacro " + file_path + " > " + TMP_URDF_PATH;
    if (system(command.toUtf8().constData()) != EXIT_SUCCESS) {
      QMessageBox::warning(this, kError, "Failed to convert XACRO to URDF.");
      return;
    }

    // Parse the URDF.
    if (!vm_.loadRobot(TMP_URDF_PATH)) {
      QMessageBox::warning(this, kError, "Failed to parse XACRO.");
      return;
    }

    // Clear the save path because overwriting XACRO with URDF is unsafe.
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

void UrdfBuilderPanel::onSaveButtonClicked()
{
  qDebug() << "UrdfBuilderPanel::onSaveButtonClicked";

  const auto cur_urdf_path = ui_->Path->text();

  if (cur_urdf_path.isEmpty()) {
    onSaveAsButtonClicked();
    return;
  }

  if (!isValid()) {
    return;
  }

  if (!saveURDF(cur_urdf_path)) {
    return;
  }
}

void UrdfBuilderPanel::onSaveAsButtonClicked()
{
  qDebug() << "UrdfBuilderPanel::onSaveAsButtonClicked";

  if (!isValid()) {
    return;
  }

  const auto last_opened_dir = getLastOpenedDir();
  SaveUrdfDialog dialog(this, last_opened_dir);
  if (dialog.exec() != QDialog::Accepted) {
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

void UrdfBuilderPanel::onEnableVisualCheckBoxToggled(bool checked)
{
  qDebug().nospace() << "UrdfBuilderPanel::onEnableVisualCheckBoxToggled(" << checked << ")";

  ogre_ctrl_->setVisualVisible(checked);
}

void UrdfBuilderPanel::onEnableCollisionCheckBoxToggled(bool checked)
{
  qDebug().nospace() << "UrdfBuilderPanel::EnableCollisiolCheckBoxToggled(" << checked << ")";

  ogre_ctrl_->setCollisionVisible(checked);
}

void UrdfBuilderPanel::onEnableInertiaCheckBoxToggled(bool checked)
{
  qDebug().nospace() << "UrdfBuilderPanel::onEnableInertiaCheckBoxToggled(" << checked << ")";

  ogre_ctrl_->setInertiaVisible(checked);
}

void UrdfBuilderPanel::onLinkTreeWidgetItemClicked(QTreeWidgetItem* item, int)
{
  qDebug() << "UrdfBuilderPanel::onLinkTreeWidgetItemClicked";

  reflectSelectedItem(item);
}

void UrdfBuilderPanel::onLinkTreeWidgetItemChanged(QTreeWidgetItem* item, int)
{
  qDebug() << "UrdfBuilderPanel::onLinkTreeWidgetItemChanged";

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

void UrdfBuilderPanel::onLinkTreeContextMenuRequested(const QPoint& point)
{
  qDebug() << "UrdfBuilderPanel::onLinkTreeContextMenuRequested";

  QMenu menu(this);
  menu.addAction(ui_->AddLinkAction);
  menu.addAction(ui_->CloneLinkAction);
  menu.addAction(ui_->RemoveLinkAction);
  menu.exec(ui_->LinkTreeWidget->mapToGlobal(point));
}

void UrdfBuilderPanel::onAddLinkActionToggled(bool)
{
  qDebug() << "UrdfBuilderPanel::onAddLinkActionToggled";

  // Allow adding links only when the root link exists.
  if (!vm_.rootLinkViewModel()) {
    QMessageBox::warning(this, kError, "Please create a new robot model or load one first.");
    return;
  }

  const auto link_vm = std::make_shared<view_model::LinkViewModel>(nullptr);
  AddLinkDialog dialog(this, vm_.linkNames(), *link_vm);
  const auto result = dialog.exec();

  if (result != QDialog::Accepted) {
    return;
  }

  vm_.addLink(link_vm);
  reload();
}

void UrdfBuilderPanel::onRemoveLinkActionToggled(bool)
{
  qDebug() << "UrdfBuilderPanel::onRemoveLinkActionToggled";

  const auto& items = ui_->LinkTreeWidget->selectedItems();
  if (items.empty()) {
    QMessageBox::warning(this, kError, "No link is selected.");
    return;
  }

  const auto front = boost::polymorphic_downcast<LinkTreeWidgetItem*>(items.front());

  // Prevent deleting the root link.
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

void UrdfBuilderPanel::onCloneLinkActionToggled(bool)
{
  qDebug() << "UrdfBuilderPanel::onCloneLinkActionToggled";

  const auto& items = ui_->LinkTreeWidget->selectedItems();
  if (items.empty()) {
    QMessageBox::warning(this, kError, "No link is selected.");
    return;
  }

  const auto front = boost::polymorphic_downcast<LinkTreeWidgetItem*>(items.front());

  // The root link cannot be duplicated.
  const auto& link = front->viewModel()->model();
  const auto& root_link = vm_.rootLinkViewModel()->model();
  if (link == root_link) {
    QMessageBox::warning(this, kError, "Root link cannot be cloned.");
    return;
  }

  vm_.cloneLink(front->viewModel());
  reload();
}

void UrdfBuilderPanel::onUpdate()
{
  ogre_ctrl_->update();
}

void UrdfBuilderPanel::onLinkDialogChanged()
{
  qDebug() << "UrdfBuilderPanel::onLinkDialogChanged";

  vm_.updateLink(old_link_vm_, link_dialog_->viewModel());
  old_link_vm_ = link_dialog_->viewModel()->clone();  // Keep the settings from the last time the URDF was updated.
  reload();
}

QString UrdfBuilderPanel::getLastOpenedDir()
{
  return settings_store_.value(kConfigKey_LastOpenedDir, QDir::homePath()).toString();
}

void UrdfBuilderPanel::setLastOpenedDir(const QString& file_path)
{
  const auto dir = QFileInfo(file_path).absolutePath();
  settings_store_.setValue(kConfigKey_LastOpenedDir, dir);
}

void UrdfBuilderPanel::defineConnections()
{
  connect(ui_->RobotName, &QLineEdit::textChanged, this, &self::onRobotNameTextChanged);

  connect(ui_->LoadButton, &QPushButton::released, this, &self::onLoadButtonClicked);
  connect(ui_->NewButton, &QPushButton::released, this, &self::onNewButtonClicked);
  connect(ui_->SaveButton, &QPushButton::released, this, &self::onSaveButtonClicked);
  connect(ui_->SaveAsButton, &QPushButton::released, this, &self::onSaveAsButtonClicked);

  connect(ui_->EnableVisualCheckBox, &QCheckBox::toggled, this, &self::onEnableVisualCheckBoxToggled);
  connect(ui_->EnableCollisionCheckBox, &QCheckBox::toggled, this, &self::onEnableCollisionCheckBoxToggled);
  connect(ui_->EnableInertiaCheckBox, &QCheckBox::toggled, this, &self::onEnableInertiaCheckBoxToggled);

  connect(ui_->LinkTreeWidget, &QTreeWidget::itemClicked, this, &self::onLinkTreeWidgetItemClicked);
  connect(ui_->LinkTreeWidget, &QTreeWidget::itemChanged, this, &self::onLinkTreeWidgetItemChanged);
  connect(ui_->LinkTreeWidget, &QTreeWidget::customContextMenuRequested, this, &self::onLinkTreeContextMenuRequested);

  connect(ui_->AddLinkAction, &QAction::triggered, this, &self::onAddLinkActionToggled);
  connect(ui_->RemoveLinkAction, &QAction::triggered, this, &self::onRemoveLinkActionToggled);
  connect(ui_->CloneLinkAction, &QAction::triggered, this, &self::onCloneLinkActionToggled);

  connect(&update_timer_, &QTimer::timeout, this, &self::onUpdate);
  connect(link_dialog_, &UpdateLinkDialog::Changed, this, &self::onLinkDialogChanged);
}

void UrdfBuilderPanel::reload()
{
  qDebug() << "UrdfBuilderPanel::reload";

  reloadLinkTree();
  reloadRobot();
}

void UrdfBuilderPanel::reloadLinkTree()
{
  const QSignalBlocker block(ui_->LinkTreeWidget);

  // Get the selected link name.
  QString selected_link_name = "";
  const auto& selected_items = ui_->LinkTreeWidget->selectedItems();
  if (!selected_items.empty()) {
    const auto front = boost::polymorphic_downcast<LinkTreeWidgetItem*>(selected_items.front());
    selected_link_name = front->viewModel()->name();
  }

  // Get the checked state.
  QSet<QString> unchecked_links;
  for (int i = 0; i < ui_->LinkTreeWidget->topLevelItemCount(); ++i) {
    collectUncheckedLinks(ui_->LinkTreeWidget->topLevelItem(i), unchecked_links);
  }

  // Delete all nodes once.
  ui_->LinkTreeWidget->clear();

  std::queue<std::pair<view_model::LinkViewModelPtr, QTreeWidgetItem*>> que;
  que.push({ vm_.rootLinkViewModel(), new LinkTreeWidgetItem(vm_.rootLinkViewModel(), ui_->LinkTreeWidget) });

  while (!que.empty()) {
    const auto t = que.front();
    que.pop();

    const auto& link_vm = t.first;
    const auto& item = t.second;

    item->setText(0, link_vm->name());
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);

    // Select the link again if the selected link still exists.
    item->setSelected(link_vm->name() == selected_link_name);

    // Keep the checked state.
    if (unchecked_links.contains(link_vm->name())) {
      item->setCheckState(0, Qt::Unchecked);
    }
    else {
      item->setCheckState(0, Qt::Checked);
    }

    // Add child nodes to the queue.
    for (const auto& child : link_vm->children()) {
      const auto child_item = new LinkTreeWidgetItem(child);
      item->addChild(child_item);
      que.push({ child, child_item });
    }
  }

  ui_->LinkTreeWidget->expandAll();
}

void UrdfBuilderPanel::reloadRobot()
{
  ogre_ctrl_->reload(vm_);
}

void UrdfBuilderPanel::selectRootLink()
{
  const auto root_item = ui_->LinkTreeWidget->topLevelItem(0);
  selectLink(root_item);
}

void UrdfBuilderPanel::selectLink(QTreeWidgetItem* item)
{
  ui_->LinkTreeWidget->clearSelection();
  item->setSelected(true);
  reflectSelectedItem(item);
}

void UrdfBuilderPanel::reflectSelectedItem(QTreeWidgetItem* item)
{
  const auto link_item = boost::polymorphic_downcast<LinkTreeWidgetItem*>(item);
  const auto& link_vm = link_item->viewModel();
  const auto link_name = link_vm->name().toStdString();

  ogre_ctrl_->unhighlightAll();
  ogre_ctrl_->highlight(link_name);

  link_dialog_->show();
  link_dialog_->readFromVM(link_vm);  // Update dialog values from the link View Model.
  old_link_vm_ = link_vm->clone();    // Keep the settings from when the link was selected.

  // Make it uneditable if it is the root link.
  link_dialog_->setTabsEnabled(link_name != vm_.rootLink()->name);
}

void UrdfBuilderPanel::addRootLink()
{
  const auto link_vm = std::make_shared<view_model::LinkViewModel>(nullptr);
  link_vm->name("root");
  vm_.addLink(link_vm);
}

bool UrdfBuilderPanel::saveURDF(const QString& file_path)
{
  if (!vm_.saveRobot(file_path)) {
    QMessageBox::warning(this, kError, "Failed to save URDF.");
    return false;
  }

  return true;
}

bool UrdfBuilderPanel::isValid()
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

bool UrdfBuilderPanel::isRobotNameValid()
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

bool UrdfBuilderPanel::isJointsValid()
{
  for (const auto& joint_pair : vm_.joints()) {
    const auto& name = joint_pair.first;
    const auto& joint = joint_pair.second;

    // Report an error if the movable joint axis is not set.
    const auto& type = joint->type;
    const auto& axis = joint->axis;

    if (
      type == ::urdf::Joint::REVOLUTE || type == ::urdf::Joint::CONTINUOUS || type == ::urdf::Joint::PRISMATIC ||
      type == ::urdf::Joint::PLANAR) {
      if (axis.x == 0 && axis.y == 0 && axis.z == 0) {
        QMessageBox::warning(this, kError, "Please set the axis of the joint '" + QString::fromStdString(name) + "'.");
        return false;
      }
    }
  }

  return true;
}

void UrdfBuilderPanel::collectUncheckedLinks(QTreeWidgetItem* item, QSet<QString>& set)
{
  if (item->checkState(0) == Qt::Unchecked) {
    const auto link_name = item->text(0);
    set.insert(link_name);
  }

  // Traverse child items.
  for (int i = 0; i < item->childCount(); ++i) {
    collectUncheckedLinks(item->child(i), set);
  }
}
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas

// A class derived from `rviz_common::Panel` can be embedded in the RViz window.
PLUGINLIB_EXPORT_CLASS(tobas::gui::ub::ui::UrdfBuilderPanel, rviz_common::Panel)
