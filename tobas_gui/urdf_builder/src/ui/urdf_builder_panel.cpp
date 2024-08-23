#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include <tobas_std_tools/console.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>

#include "../../include/urdf_builder/ui/urdf_builder_panel.hpp"
#include "../../include/urdf_builder/ui/update_link_dialog.hpp"
#include "../../include/urdf_builder/ui/add_link_dialog.hpp"
#include "../../include/urdf_builder/ui/widget_item.hpp"
#include "../../include/urdf_builder/ui/save_urdf_dialog.hpp"
#include "../../include/urdf_builder/ogre_helpers/static_link_updater.hpp"
#include "../../include/urdf_builder/utils/constants.hpp"
#include "ui_urdf_builder_panel.h"

#define ROBOT_MODEL_UPDATE_INTERVAL 5
#define INVALID_CHARS " '\"#$%&()^~|,.<>/\\!?"
#define TMP_URDF_PATH "/tmp/urdf_builder.urdf"

using namespace std;

namespace urdf_builder
{
namespace ui
{
URDFBuilderPanel::URDFBuilderPanel(QWidget* parent)
  : rviz_common::Panel(parent), ui_(new Ui::URDFBuilderPanelUI()), ogre_ctrl_(nullptr)
{
  ui_->setupUi(this);

  ui_->EnableVisualCheckBox->setChecked(kDefaultVisualVisible);
  ui_->EnableCollisionCheckBox->setChecked(kDefaultCollisionVisible);

  update_timer_ = new QTimer();

  link_dialog_ = new UpdateLinkDialog(this);
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

  const auto context = getDisplayContext();
  const auto node = context->getRosNodeAbstraction().lock()->get_raw_node();

  link_dialog_->onInitialize(node);
  property_client_ = make_shared<ptree::PropertyClient>(node, tobas::kPropertyServerGCS, kPropertySection);
  ogre_ctrl_ = make_shared<ogre_helpers::OgreController>(context);
  update_timer_->start(ROBOT_MODEL_UPDATE_INTERVAL);
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
  PRINT_DEBUG("URDFBuilderPanel::RobotNameTextChanged");

  vm_.name(name.toStdString());
}

void URDFBuilderPanel::NewButtonClicked()
{
  PRINT_DEBUG("URDFBuilderPanel::NewButtonClicked");

  vm_.newRobot();
  ui_->Path->setText("");
  ui_->RobotName->clear();
  addRootLink();

  reload();
}

void URDFBuilderPanel::LoadButtonClicked()
{
  PRINT_DEBUG("URDFBuilderPanel::LoadButtonClicked");

  // URDFまたはXACROのパスを取得
  const auto last_opened_dir = getLastOpenedDir();
  const auto file_path = QFileDialog::getOpenFileName(
    this, tr("Load URDF"), QString::fromStdString(last_opened_dir),
    tr("Robot Description (*.urdf *.xacro);;All Files (*)"));

  if (file_path.isEmpty())
    return;

  setLastOpenedDir(file_path.toStdString());

  if (file_path.endsWith(".urdf"))
  {
    // URDFを解析
    if (!vm_.loadRobot(file_path))
    {
      QMessageBox::warning(this, kError, "Failed to parse URDF.");
      return;
    }

    // URDFのパスを設定
    ui_->Path->setText(file_path);
  }
  else if (file_path.endsWith(".xacro"))
  {
    // XACROを展開
    const auto command = "xacro " + file_path + " > " + TMP_URDF_PATH;
    if (system(command.toUtf8()) != 0)
    {
      QMessageBox::warning(this, kError, "Failed to convert XACRO to URDF.");
      return;
    }

    // URDFを解析
    if (!vm_.loadRobot(TMP_URDF_PATH))
    {
      QMessageBox::warning(this, kError, "Failed to parse XACRO.");
      return;
    }

    // XACROをURDFで上書きするのはまずいため保存用パスを消去
    ui_->Path->clear();
  }
  else
  {
    QMessageBox::warning(this, kError, "Invalid file format: " + file_path);
    return;
  }

  ui_->RobotName->setText(QString::fromStdString(vm_.name()));

  reload();
}

void URDFBuilderPanel::SaveButtonClicked()
{
  PRINT_DEBUG("URDFBuilderPanel::SaveButtonClicked");

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
  PRINT_DEBUG("URDFBuilderPanel::SaveAsButtonClicked");

  if (!isValid())
    return;

  const auto last_opened_dir = getLastOpenedDir();
  SaveUrdfDialog dialog(this, QString::fromStdString(last_opened_dir));

  const auto result = dialog.exec();
  if (result != QDialog::Accepted)
    return;
  const auto file_path = dialog.selectedFiles().first();
  assert(file_path.endsWith(".urdf"));

  setLastOpenedDir(file_path.toStdString());

  if (!saveURDF(file_path))
    return;

  ui_->Path->setText(file_path);
}

void URDFBuilderPanel::EnableVisualCheckBoxToggled(bool ckecked)
{
  PRINT_DEBUG("URDFBuilderPanel::EnableVisualCheckBoxToggled(" << ckecked << ")");

  ogre_ctrl_->setVisualVisible(ckecked);
}

void URDFBuilderPanel::EnableCollisionCheckBoxToggled(bool ckecked)
{
  PRINT_DEBUG("URDFBuilderPanel::EnableCollisiolCheckBoxToggled(" << ckecked << ")");

  ogre_ctrl_->setCollisionVisible(ckecked);
}

void URDFBuilderPanel::LinkTreeWidgetItemClicked(QTreeWidgetItem* item, int)
{
  PRINT_DEBUG("URDFBuilderPanel::LinkTreeWidgetItemClicked");

  const auto link_item = dynamic_cast<LinkTreeWidgetItem*>(item);
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

void URDFBuilderPanel::LinkTreeWidgetItemChanged(QTreeWidgetItem* item, int)
{
  PRINT_DEBUG("URDFBuilderPanel::LinkTreeWidgetItemChanged");

  const auto link_item = dynamic_cast<LinkTreeWidgetItem*>(item);
  const auto link_name = link_item->viewModel()->name().toStdString();

  if (item->checkState(0) == Qt::Unchecked)
    ogre_ctrl_->hide(link_name);
  else
    ogre_ctrl_->show(link_name);
}

void URDFBuilderPanel::LinkTreeContextMenuRequested(const QPoint& point)
{
  PRINT_DEBUG("URDFBuilderPanel::LinkTreeContextMenuRequested");

  QMenu menu(this);
  menu.addAction(ui_->AddLinkAction);
  menu.addAction(ui_->CloneLinkAction);
  menu.addAction(ui_->RemoveLinkAction);
  menu.exec(ui_->LinkTreeWidget->mapToGlobal(point));
}

void URDFBuilderPanel::AddLinkActionToggled(bool)
{
  PRINT_DEBUG("URDFBuilderPanel::AddLinkActionToggled");

  // ルートリンクが存在する場合のみリンクの追加を許可
  if (vm_.rootLinkViewModel() == nullptr)
  {
    QMessageBox::warning(this, kError, "Please create a new robot model or load one first.");
    return;
  }

  const auto link_vm = make_shared<view_model::LinkViewModel>(nullptr);
  AddLinkDialog dialog(this, vm_.linkNames(), *link_vm);
  const auto result = dialog.exec();

  if (result != QDialog::Accepted)
    return;

  vm_.addLink(link_vm);
  reload();
}

void URDFBuilderPanel::RemoveLinkActionToggled(bool)
{
  PRINT_DEBUG("URDFBuilderPanel::RemoveLinkActionToggled");

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
  PRINT_DEBUG("URDFBuilderPanel::CloneLinkActionToggled");

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
  PRINT_DEBUG("URDFBuilderPanel::LinkDialogChanged");

  vm_.updateLink(old_link_vm_, link_dialog_->viewModel());
  old_link_vm_ = link_dialog_->viewModel()->clone();  // 最後にURDFが更新されたときの設定を保持
  reload();
}

string URDFBuilderPanel::getLastOpenedDir()
{
  string res;
  if (property_client_->get(kConfigKey_LastOpenedDir, res) < 0)
  {
    PRINT_WARN(property_client_->errorMessage());
    res = linux::homeDir();
  }
  return res;
}

void URDFBuilderPanel::setLastOpenedDir(const string& file_path)
{
  boost::filesystem::path p(file_path);
  const auto dir = p.parent_path().string();

  if (property_client_->set(kConfigKey_LastOpenedDir, dir) < 0)
  {
    PRINT_WARN(property_client_->errorMessage());
    return;
  }
  if (property_client_->save() < 0)
  {
    PRINT_WARN(property_client_->errorMessage());
    return;
  }
}

void URDFBuilderPanel::defineConnections()
{
  connect(ui_->RobotName, SIGNAL(textChanged(const QString&)), this, SLOT(RobotNameTextChanged(const QString&)));

  connect(ui_->LoadButton, SIGNAL(released()), this, SLOT(LoadButtonClicked()));
  connect(ui_->NewButton, SIGNAL(released()), this, SLOT(NewButtonClicked()));
  connect(ui_->SaveButton, SIGNAL(released()), this, SLOT(SaveButtonClicked()));
  connect(ui_->SaveAsButton, SIGNAL(released()), this, SLOT(SaveAsButtonClicked()));

  connect(ui_->EnableVisualCheckBox, SIGNAL(toggled(bool)), this, SLOT(EnableVisualCheckBoxToggled(bool)));
  connect(ui_->EnableCollisionCheckBox, SIGNAL(toggled(bool)), this, SLOT(EnableCollisionCheckBoxToggled(bool)));

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
  connect(ui_->RemoveLinkAction, SIGNAL(triggered(bool)), this, SLOT(RemoveLinkActionToggled(bool)));
  connect(ui_->CloneLinkAction, SIGNAL(triggered(bool)), this, SLOT(CloneLinkActionToggled(bool)));

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(OnUpdate()));
  connect(link_dialog_, SIGNAL(Changed()), this, SLOT(LinkDialogChanged()));
}

void URDFBuilderPanel::reload()
{
  PRINT_DEBUG("URDFBuilderPanel::reload");

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
  que.push({ vm_.rootLinkViewModel(), new LinkTreeWidgetItem(vm_.rootLinkViewModel(), ui_->LinkTreeWidget) });

  while (!que.empty())
  {
    const auto t = que.front();
    que.pop();

    const auto& link_vm = t.first;
    const auto& item = t.second;

    item->setText(0, link_vm->name());
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);

    // 選択リンクが残っている場合は再び選択
    item->setSelected(link_vm->name() == selected_link_name);

    // チェック状態を保持
    if (unchecked_links.find(link_vm->name().toStdString()) != unchecked_links.end())
      item->setCheckState(0, Qt::Unchecked);
    else
      item->setCheckState(0, Qt::Checked);

    // 子ノードをキューに追加
    for (const auto& child : link_vm->children())
    {
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

void URDFBuilderPanel::addRootLink()
{
  const auto link_vm = make_shared<view_model::LinkViewModel>(nullptr);
  link_vm->name("root");
  vm_.addLink(link_vm);
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
      type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC
      || type == urdf::Joint::PLANAR)
    {
      if (axis.x == 0 && axis.y == 0 && axis.z == 0)
      {
        QMessageBox::warning(this, kError, "Please set the axis of the joint '" + QString::fromStdString(name) + "'.");
        return false;
      }
    }
  }

  return true;
}

void URDFBuilderPanel::collectUncheckedLinks(QTreeWidgetItem* item, unordered_set<string>& set)
{
  if (item->checkState(0) == Qt::Unchecked)
  {
    const auto link_name = item->text(0).toStdString();
    set.insert(link_name);
  }

  // 子アイテムを走査
  for (int i = 0; i < item->childCount(); ++i)
    collectUncheckedLinks(item->child(i), set);
}
}  // namespace ui
}  // namespace urdf_builder

// rviz_common::Panelの派生クラスならばRvizのメインウィジェットにプラグインできる
PLUGINLIB_EXPORT_CLASS(urdf_builder::ui::URDFBuilderPanel, rviz_common::Panel)
