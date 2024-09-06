#include <QButtonGroup>
#include <QCheckBox>
#include <QStackedWidget>

#include "tobas_setup_assistant/start/start.hpp"

namespace gui
{
namespace setup_assistant
{
StartWidget::StartWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings)
{
  const auto ckb_group = new QButtonGroup(this);  // コンストラクタで解放されないように親ウィジェットを設定
  const auto stack = new QStackedWidget();

  const auto new_ckb = new QCheckBox("Create new Tobas configuration package");
  urdf_loader_ = new URDFLoaderWidget(node, robot, settings);
  ckb_group->addButton(new_ckb);
  ckb_group->setId(new_ckb, 0);
  stack->addWidget(urdf_loader_);

  const auto edit_ckb = new QCheckBox("Edit existing Tobas configuration package");
  package_loader_ = new PackageLoaderWidget(node, robot, settings);
  ckb_group->addButton(edit_ckb);
  ckb_group->setId(edit_ckb, 1);
  stack->addWidget(package_loader_);

  new_ckb->setChecked(true);      // デフォルト
  ckb_group->setExclusive(true);  // 1つのみ有効

  // Layout
  const auto rows = new QVBoxLayout(this);
  rows->addWidget(new_ckb);
  rows->addWidget(edit_ckb);
  rows->addWidget(stack);

  // Connections
  connect(ckb_group, &QButtonGroup::idClicked, stack, &QStackedWidget::setCurrentIndex);
}
}  // namespace setup_assistant
}  // namespace gui
