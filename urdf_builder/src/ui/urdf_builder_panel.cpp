#include <QListWidget>
#include <QFileDialog>
#include <QTreeWidget>
#include <QtWidgets/QtWidgets>
#include <QGridLayout>
#include <ros/ros.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <tobas_std_tools/fstream.hpp>

#include "../../include/urdf_builder/ui/urdf_builder_panel.hpp"
#include "../../include/urdf_builder/ui/update_link_dialog.hpp"
#include "../../include/urdf_builder/ui/add_link_dialog.hpp"
#include "../../include/urdf_builder/ui/widget_item.hpp"
#include "../../include/urdf_builder/rviz_helpers/display_context_proxy.hpp"
#include "../../include/urdf_builder/rviz_helpers/static_link_updater.hpp"
#include "../../include/urdf_builder/utils/constants.hpp"
#include "ui_urdf_builder_panel.h"

#define ROBOT_MODEL_UPDATE_INTERVAL 5
#define INVALID_CHARS " '\"#$%&()^~|,.<>/\\!?"
#define TMP_URDF_PATH "/tmp/urdf_builder.urdf"

using namespace std;
using namespace urdf;

namespace urdf_builder
{
namespace ui
{
URDFBuilderPanel::URDFBuilderPanel(QWidget* item)
  : rviz::Panel(item),
    config_path_(tobas_std::expandPath(kConfigPath)),
    ui_(new Ui::URDFBuilderPanelUI()),
    ogre_ctrl_(nullptr)
{
  // configファイルを作成
  if (!tobas_std::fileExists(config_path_))
    createConfig();

  ui_->setupUi(this);

  ui_->EnableVisualCheckBox->setChecked(kDefaultVisualVisible);
  ui_->EnableCollisionCheckBox->setChecked(kDefaultCollisionVisible);

  update_timer_ = new QTimer();

  const auto& vm = make_shared<view_model::LinkViewModel>(nullptr);
  link_dialog_ = new UpdateLinkDialog(vm);
  link_dialog_->hide();
  ui_->scrollAreaWidgetContents->layout()->addWidget(link_dialog_);

  defineConnections();
}

URDFBuilderPanel::~URDFBuilderPanel()
{
  update_timer_->stop();
  delete update_timer_;
  delete link_dialog_;
}

void URDFBuilderPanel::onInitialize()
{
  Panel::onInitialize();

  ogre_ctrl_.reset(new ogre_helpers::OgreController(vis_manager_));
  update_timer_->start(ROBOT_MODEL_UPDATE_INTERVAL);
}

void URDFBuilderPanel::load(const rviz::Config& config)
{
  Panel::load(config);
}

void URDFBuilderPanel::save(rviz::Config config) const
{
  Panel::save(config);
}

void URDFBuilderPanel::RobotNameTextChanged(const QString& name)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::RobotNameTextChanged");

  vm_.name(name.toStdString());
}

void URDFBuilderPanel::NewButtonClicked()
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::NewButtonClicked");

  vm_.newRobot();
  ui_->Path->setText("");
  ui_->RobotName->clear();
  addRootLink();

  reload();
}

void URDFBuilderPanel::LoadButtonClicked()
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::LoadButtonClicked");

  const auto last_opened_dir = getLastOpenedDir();
  auto file_path = QFileDialog::getOpenFileName(
    this, tr("Load URDF"), QString::fromStdString(last_opened_dir),
    tr("Robot Description (*.urdf *.xacro);;All Files (*)"));

  if (file_path.isEmpty())
    return;

  setLastOpenedDir(file_path.toStdString());

  // xacroが指定された場合は展開する
  if (file_path.endsWith(".xacro"))
  {
    const auto command = "xacro " + file_path + " > " + TMP_URDF_PATH;
    if (system(command.toUtf8()) != 0)
    {
      QMessageBox::warning(this, kError, "Failed to convert XACRO to URDF.");
      return;
    }
    file_path = TMP_URDF_PATH;
  }

  if (!vm_.loadRobot(file_path))
  {
    QMessageBox::warning(this, kError, "Failed to parse URDF.");
    return;
  }

  ui_->Path->setText(file_path);
  ui_->RobotName->setText(QString::fromStdString(vm_.name()));

  reload();
}

void URDFBuilderPanel::SaveButtonClicked()
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::SaveButtonClicked");

  if (!isValid())
    return;

  if (ui_->Path->text().isEmpty())
  {
    SaveAsButtonClicked();
    return;
  }

  saveURDF(ui_->Path->text());
}

void URDFBuilderPanel::SaveAsButtonClicked()
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::SaveAsButtonClicked");

  if (!isValid())
    return;

  const auto last_opened_dir = getLastOpenedDir();
  const auto file_path = QFileDialog::getSaveFileName(
    this, tr("save URDF"), QString::fromStdString(last_opened_dir),
    tr("URDF (*.urdf);;All Files (*)"));

  if (file_path.isEmpty())
    return;

  setLastOpenedDir(file_path.toStdString());

  if (!saveURDF(file_path))
    return;

  ui_->Path->setText(file_path);
}

void URDFBuilderPanel::EnableVisualCheckBoxToggled(bool ckecked)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::EnableVisualCheckBoxToggled(" << ckecked << ")");

  ogre_ctrl_->setVisualVisible(ckecked);
}

void URDFBuilderPanel::EnableCollisionCheckBoxToggled(bool ckecked)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::EnableCollisiolCheckBoxToggled(" << ckecked << ")");

  ogre_ctrl_->setCollisionVisible(ckecked);
}

void URDFBuilderPanel::LinkTreeWidgetItemClicked(QTreeWidgetItem* item, int)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::LinkTreeWidgetItemClicked");

  const auto link_item = dynamic_cast<LinkTreeWidgetItem*>(item);
  const auto link_name = link_item->viewModel()->name().toStdString();

  ogre_ctrl_->unhighlightAll();
  ogre_ctrl_->highlight(link_name);

  const auto vm = link_item->viewModel()->clone();
  vm->usedLinkNames(vm_.linkNames());

  link_dialog_->show();
  link_dialog_->readFromVM(vm);  // リンクのビューモデルからダイアログの値を更新
  old_link_vm_ = vm->clone();    // リンクが選択された時点での設定を保持

  // ルートリンクだったら変更不可にする
  if (link_name == vm_.rootLink()->name)
    link_dialog_->setTabsEnabled(false);
  else
    link_dialog_->setTabsEnabled(true);
}

void URDFBuilderPanel::LinkTreeWidgetItemChanged(QTreeWidgetItem* item, int)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::LinkTreeWidgetItemChanged");

  const auto link_item = dynamic_cast<LinkTreeWidgetItem*>(item);
  const auto link_name = link_item->viewModel()->name().toStdString();

  if (item->checkState(0) == Qt::Unchecked)
    ogre_ctrl_->hide(link_name);
  else
    ogre_ctrl_->show(link_name);
}

void URDFBuilderPanel::LinkTreeContextMenuRequested(const QPoint& point)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::LinkTreeContextMenuRequested");

  QMenu menu(this);
  menu.addAction(ui_->AddLinkAction);
  menu.addAction(ui_->CloneLinkAction);
  menu.addAction(ui_->RemoveLinkAction);
  menu.exec(ui_->LinkTreeWidget->mapToGlobal(point));
}

void URDFBuilderPanel::AddLinkActionToggled(bool)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::AddLinkActionToggled");

  // ルートリンクが存在する場合のみリンクの追加を許可
  if (vm_.rootLinkViewModel() == nullptr)
  {
    QMessageBox::warning(this, kError, "Please create a new robot model or load one first.");
    return;
  }

  const auto link_vm = make_shared<view_model::LinkViewModel>(nullptr);
  link_vm->usedLinkNames(vm_.linkNames());

  AddLinkDialog dialog(link_vm);
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  vm_.addLink(link_vm);
  reload();
}

void URDFBuilderPanel::RemoveLinkActionToggled(bool)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::RemoveLinkActionToggled");

  const auto& items = ui_->LinkTreeWidget->selectedItems();
  if (items.empty())
  {
    QMessageBox::warning(this, kError, "No link is selected.");
    return;
  }

  const auto& front = dynamic_cast<LinkTreeWidgetItem*>(items.front());

  // ルートリンクは消せないようにする
  const auto& link = front->viewModel()->model();
  const auto& root_link = vm_.rootLinkViewModel()->model();
  if (link == root_link)
  {
    QMessageBox::warning(this, kError, "Root link cannot be removed.");
    return;
  }

  vm_.removeLink(front->viewModel());
  reload();
  link_dialog_->hide();
}

void URDFBuilderPanel::CloneLinkActionToggled(bool)
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::CloneLinkActionToggled");

  const auto& items = ui_->LinkTreeWidget->selectedItems();
  if (items.empty())
  {
    QMessageBox::warning(this, kError, "No link is selected.");
    return;
  }

  const auto& front = dynamic_cast<LinkTreeWidgetItem*>(items.front());

  // ルートリンクは複製不可
  const auto& link = front->viewModel()->model();
  const auto& root_link = vm_.rootLinkViewModel()->model();
  if (link == root_link)
  {
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
  ROS_DEBUG_STREAM("URDFBuilderPanel::LinkDialogChanged");

  vm_.updateLink(old_link_vm_, link_dialog_->viewModel());
  old_link_vm_ = link_dialog_->viewModel()->clone();  // 最後にURDFが更新されたときの設定を保持
  reload();
}

void URDFBuilderPanel::createConfig()
{
  // ディレクトリを作成
  boost::filesystem::path dir(getenv("HOME"));
  dir /= ".config/urdf_builder";
  boost::filesystem::create_directories(dir);

  // 各キーのデフォルト値を設定
  boost::property_tree::ptree pt_;
  pt_.put(kConfigKey_LastOpenedDir, getenv("HOME"));

  // configを保存
  boost::property_tree::ini_parser::write_ini(config_path_, pt_);
}

string URDFBuilderPanel::getLastOpenedDir()
{
  boost::property_tree::ini_parser::read_ini(config_path_, pt_);
  return pt_.get<string>(kConfigKey_LastOpenedDir);
}

void URDFBuilderPanel::setLastOpenedDir(const string& file_path)
{
  boost::filesystem::path p(file_path);
  const auto dir = p.parent_path().string();
  pt_.put(kConfigKey_LastOpenedDir, dir);
  boost::property_tree::ini_parser::write_ini(config_path_, pt_);
}

void URDFBuilderPanel::defineConnections()
{
  connect(
    ui_->RobotName, SIGNAL(textChanged(const QString&)), this,
    SLOT(RobotNameTextChanged(const QString&)));

  connect(ui_->LoadButton, SIGNAL(released()), this, SLOT(LoadButtonClicked()));
  connect(ui_->NewButton, SIGNAL(released()), this, SLOT(NewButtonClicked()));
  connect(ui_->SaveButton, SIGNAL(released()), this, SLOT(SaveButtonClicked()));
  connect(ui_->SaveAsButton, SIGNAL(released()), this, SLOT(SaveAsButtonClicked()));

  connect(
    ui_->EnableVisualCheckBox, SIGNAL(toggled(bool)), this,
    SLOT(EnableVisualCheckBoxToggled(bool)));
  connect(
    ui_->EnableCollisionCheckBox, SIGNAL(toggled(bool)), this,
    SLOT(EnableCollisionCheckBoxToggled(bool)));

  connect(
    ui_->LinkTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this,
    SLOT(LinkTreeWidgetItemClicked(QTreeWidgetItem*, int)));
  connect(
    ui_->LinkTreeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
    SLOT(LinkTreeWidgetItemChanged(QTreeWidgetItem*, int)));
  connect(
    ui_->LinkTreeWidget, SIGNAL(customContextMenuRequested(const QPoint&)), this,
    SLOT(LinkTreeContextMenuRequested(const QPoint&)));

  connect(ui_->AddLinkAction, SIGNAL(triggered(bool)), this, SLOT(AddLinkActionToggled(bool)));
  connect(
    ui_->RemoveLinkAction, SIGNAL(triggered(bool)), this, SLOT(RemoveLinkActionToggled(bool)));
  connect(ui_->CloneLinkAction, SIGNAL(triggered(bool)), this, SLOT(CloneLinkActionToggled(bool)));

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(OnUpdate()));
  connect(link_dialog_, SIGNAL(Changed()), this, SLOT(LinkDialogChanged()));
}

void URDFBuilderPanel::reload()
{
  ROS_DEBUG_STREAM("URDFBuilderPanel::reload");

  reloadLinkTree();
  reloadRobot();
}

void URDFBuilderPanel::reloadLinkTree()
{
  ui_->LinkTreeWidget->blockSignals(true);

  // 選択されているリンク名を取得
  QString selected_link_name = "";
  const auto& selected_items = ui_->LinkTreeWidget->selectedItems();
  if (!selected_items.empty())
  {
    const auto& front = dynamic_cast<LinkTreeWidgetItem*>(selected_items.front());
    selected_link_name = front->viewModel()->name();
  }

  // チェック状態を取得
  unordered_set<string> unchecked_links;
  for (int i = 0; i < ui_->LinkTreeWidget->topLevelItemCount(); ++i)
    collectUncheckedLinks(ui_->LinkTreeWidget->topLevelItem(i), unchecked_links);

  // 一度全てのノードをを削除
  ui_->LinkTreeWidget->clear();

  queue<pair<view_model::LinkViewModelPtr, QTreeWidgetItem*>> que;
  que.push({ vm_.rootLinkViewModel(),
             new LinkTreeWidgetItem(vm_.rootLinkViewModel(), ui_->LinkTreeWidget) });

  while (!que.empty())
  {
    const auto t = que.front();
    que.pop();

    const auto& vm = t.first;
    const auto& item = t.second;

    item->setText(0, vm->name());
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);

    // 選択リンクが残っている場合は再び選択
    if (vm->name() == selected_link_name)
      item->setSelected(true);
    else
      item->setSelected(false);

    // チェック状態を保持
    if (unchecked_links.contains(vm->name().toStdString()))
      item->setCheckState(0, Qt::Unchecked);
    else
      item->setCheckState(0, Qt::Checked);

    // 子ノードをキューに追加
    for (const auto& child : vm->children())
    {
      auto child_item = new LinkTreeWidgetItem(child);
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

void URDFBuilderPanel::addRootLink()
{
  const auto& vm = make_shared<view_model::LinkViewModel>(nullptr);
  vm->name("root");
  vm->usedLinkNames(QStringList(vm->name()));
  vm_.addLink(vm);
}

bool URDFBuilderPanel::saveURDF(const QString& file_path)
{
  if (!vm_.saveRobot(file_path))
  {
    QMessageBox::warning(this, kError, "Failed to save URDF.");
    return false;
  }

  return true;
}

bool URDFBuilderPanel::isValid()
{
  if (!isRobotNameValid())
    return false;

  if (!isJointsValid())
    return false;

  return true;
}

bool URDFBuilderPanel::isRobotNameValid()
{
  const auto name = ui_->RobotName->text();

  if (name.isEmpty())
  {
    QMessageBox::warning(this, kError, "Please set robot name.");
    return false;
  }

  for (const auto& ch : INVALID_CHARS)
  {
    if (name.contains(ch))
    {
      QMessageBox::warning(this, kError, "Robot name cannot contain '" + QString(ch) + "'.");
      return false;
    }
  }

  return true;
}

bool URDFBuilderPanel::isJointsValid()
{
  for (const auto& joint_pair : vm_.joints())
  {
    const auto& name = joint_pair.first;
    const auto& joint = joint_pair.second;

    // 可動関節の軸が設定されていなければエラー
    const auto& type = joint->type;
    const auto& axis = joint->axis;
    if (
      type == Joint::REVOLUTE || type == Joint::CONTINUOUS || type == Joint::PRISMATIC
      || type == Joint::PLANAR)
    {
      if (axis.x == 0 && axis.y == 0 && axis.z == 0)
      {
        QMessageBox::warning(
          this, kError, "Please set the axis of the joint '" + QString::fromStdString(name) + "'.");
        return false;
      }
    }
  }

  return true;
}

void URDFBuilderPanel::collectUncheckedLinks(QTreeWidgetItem* item, unordered_set<string>& set)
{
  if (item->checkState(0) == Qt::Unchecked)
    set.insert(item->text(0).toStdString());

  // 子アイテムを走査
  for (int i = 0; i < item->childCount(); ++i)
    collectUncheckedLinks(item->child(i), set);
}
}  // namespace ui
}  // namespace urdf_builder

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(urdf_builder::ui::URDFBuilderPanel, rviz::Panel)
