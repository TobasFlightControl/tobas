#include "tobas_setup_assistant/start/start.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QStackedWidget>

namespace gui
{
namespace sa
{
StartWidget::StartWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings)
{
  const auto ckb_group = new QButtonGroup(this);  // コンストラクタで解放されないように親ウィジェットを設定
  const auto stack = new QStackedWidget();

  const auto new_ckb = new QCheckBox("Create new Tobas configuration package");
  urdf_loader_ = new URDFLoaderWidget(node, robot);
  ckb_group->addButton(new_ckb);
  ckb_group->setId(new_ckb, kNewId);
  stack->addWidget(urdf_loader_);

  const auto edit_ckb = new QCheckBox("Edit existing Tobas configuration package");
  package_loader_ = new PackageLoaderWidget(node, robot, settings);
  ckb_group->addButton(edit_ckb);
  ckb_group->setId(edit_ckb, kEditId);
  stack->addWidget(package_loader_);

  new_ckb->setChecked(true);      // デフォルト
  ckb_group->setExclusive(true);  // 1つのみ有効

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(new_ckb);
  rows->addWidget(edit_ckb);
  rows->addWidget(stack);

  setLayout(rows);

  // Connection
  connect(ckb_group, &QButtonGroup::idToggled, stack, &QStackedWidget::setCurrentIndex);
}
}  // namespace sa
}  // namespace gui
